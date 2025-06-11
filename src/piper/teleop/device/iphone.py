# IPhone Teleoperation

import json
import threading
from typing import List, Tuple, Any
import time
import numpy as np
import click
import socketio
from flask import Flask

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation



class IPhoneDevice:
    """
    Returns the absolute pose of the iPhone device relative to the world frame.
    The coordinate system is defined when ARKit is initialized.

    Absolute pose coordinate system (at ARKit start):

    y (out of screen)
    ⊙---> x
    |
    |
    ↓ z


    If you compute the relative pose (T_0_inv @ T_now), the coordinate system becomes:

    z (out of screen)
    ⊙---> y
    |
    |
    ↓ x

    """

    def __init__(self, port: int = 5555, silent: bool = True):
        self._silent = silent
        self._port = port
        self._latest_transform: dict = {}
        self._latest_speed: dict = {}
        self._commands: list[str] = []

        # Use threading mode for socketio
        self._sio = socketio.Server(async_mode="threading", cors_allowed_origins="*")
        self._app = Flask(__name__)
        self._app.wsgi_app = socketio.WSGIApp(self._sio, self._app.wsgi_app)

        # Set up the event handler for updates
        @self._sio.event
        def connect(sid, environ):
            if not self._silent:
                print(f"===============>Client connected: {sid}")

        @self._sio.event
        def disconnect(sid):
            if not self._silent:
                print(f"===============>Client disconnected: {sid}")

        @self._sio.event
        def update(sid, data):
            try:
                data = json.loads(data)
                self._latest_transform = data
                self._sio.emit("commands", json.dumps(self._commands), to=sid)
            except Exception as e:
                if not self._silent:
                    print(f"Update failed: {e}")

    def _run_server(self):
        """Run the Flask server with threading."""
        self._app.run(host="0.0.0.0", port=self._port, threaded=True)

    def start(self):
        """Start the server in a background thread."""
        server_thread = threading.Thread(target=self._run_server, daemon=True)
        server_thread.start()
        if not self._silent:
            print(f"IPhone Device running at http://0.0.0.0:{self._port}")

    def stop(self):
        if not self._silent:
            print("IPhone Device stopped.")

    def get_cmd(self) -> dict:
        return self._latest_transform

    def send_cmd(self, enable: bool) -> None:
        self._commands = ["start_haptics" if enable else "stop_haptics"]
        
        
class IphoneReader:
    def __init__(self, port: int = 5555, silent: bool = True):
        self._device = IPhoneDevice(port=port, silent=silent)

        # record the initial transform
        self._init_transform = None
        self._init_transform_inverse = None
        
        # record the latest transform and timestamp
        self._latest_transform = None
        self._latest_timestamp = None

        # publishing variables
        self._velocity = [0, 0, 0]
        self._position = [0, 0, 0]


    def start(self):
        self._device.start()

    def stop(self):
        self._device.stop()

    def reset_transform(self, *args, timeout=5.0, poll_interval=0.1):
        """
        Wait until device returns data, then set the initial transform.
        
        Args:
            timeout (float): Maximum time to wait in seconds.
            poll_interval (float): Time between checks in seconds.
        """
        start_time = time.time()
        while True:
            data = self._device.get_cmd()
            if data:
                self._init_transform = np.array(data.get("transformMatrix"))
                self._init_transform_inverse = np.linalg.inv(self._init_transform)
                print("[Resetting IphoneReader] initial position", [round(v, 4) for v in self._init_transform[:3, 3].tolist()])
                self._latest_transform = self._init_transform.copy()
                self._latest_timestamp = data.get("timestamp")
                print("[IphoneReader] Initial transform reset.")
                break
            elif time.time() - start_time > timeout:
                print("Timeout: Failed to get initial transform data after waiting.")
                break
            else:
                time.sleep(poll_interval)
        
        return {"success": True, "message": "Triggered!"}

    def update_transfrom(self) -> dict:
        data = self._device.get_cmd()

        if data:
            current_transform = np.array(data.get("transformMatrix")) @ self._init_transform_inverse
            # print("current_transform", [round(v, 4) for v in current_transform.flatten().tolist()])
            current_timestamp = data.get("timestamp")

            # Check if the current timestamp is the same as the latest timestamp (No update or disconnect)
            if current_timestamp == self._latest_timestamp:
                return {
                    "transform_matrix": self._latest_transform,
                    "velocity": [0, 0, 0],
                    "position": self._latest_transform[:3, 3].tolist(),
                    "timestamp": self._latest_timestamp
                }

            if self._latest_transform is not None:
                current_position = current_transform[:3, 3]
                current_velocity = (current_position - self._latest_transform[:3, 3]) / (current_timestamp - self._latest_timestamp)
                self._velocity = current_velocity.tolist()
                self._position = current_position.tolist()
                
            self._latest_transform = current_transform.copy()
            self._latest_timestamp = current_timestamp

        return {
                "transform_matrix": self._latest_transform,
                "velocity": self._velocity,
                "position": self._position,
                "timestamp": self._latest_timestamp
                }
    def read(self) -> Tuple[np.ndarray, Any]:
        update_data = self.update_transfrom()
        return update_data.get("transform_matrix"), None
    
    def __enter__(self):
        self.start()
        print("Waiting for device to connect...press the reset button on the device to start")
        self.reset_transform(timeout=30) #
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

def main(device_only: bool = True):
    TEST_DEVICE_ONLY = device_only
    
    # Initialize the figure and 3D axes
    plt.ion()  # Turn on interactive mode
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Create two scatter plots: one for current position and one for trajectory
    current_scatter = ax.scatter([], [], [], c='b', marker='o', s=100, label='Current Position')
    trajectory_scatter = ax.scatter([], [], [], c='r', marker='.', s=20, alpha=0.5, label='Trajectory')
    
    # Set axis labels and limits
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([-0.5, 0.5])
    ax.legend()
    
    # Store position history
    position_history = []
    
    def update_plot():
        if position_history:
            # Update current position
            current_pos = position_history[-1]
            current_scatter._offsets3d = ([current_pos[0]], [current_pos[1]], [current_pos[2]])
            
            # Update trajectory
            if len(position_history) > 1:
                x = [p[0] for p in position_history]
                y = [p[1] for p in position_history]
                z = [p[2] for p in position_history]
                trajectory_scatter._offsets3d = (x, y, z)
            
            fig.canvas.draw()
            fig.canvas.flush_events()
    
    if TEST_DEVICE_ONLY:
        device = IPhoneDevice()
        device.start()

        try:
            while True:
                data = device.get_cmd()
                if data:
                    current_position = np.array(data['transformMatrix'])[:3, 3]
                    position_history.append(current_position.tolist())
                    print("Latest Device Position data:", np.round(current_position, 4))
                    update_plot()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("Stopping device...")
        finally:
            device.stop()
            plt.close(fig)
    else:
        iphone_publisher = IphoneReader(port=5555, silent=False)
        iphone_publisher.start()
        print("Waiting for device to connect...press the reset button on the device to start")
        iphone_publisher.reset_transform(timeout=30)
        
        try:
            while True:
                data = iphone_publisher.update_transfrom()
                current_position = data['transform_matrix'][:3, 3]
                position_history.append(current_position.tolist())
                print("Latest data:", np.round(current_position, 4))
                update_plot()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("Stopping device...")
        finally:
            iphone_publisher.stop()
            plt.close(fig)

        
if __name__ == "__main__":
    main()