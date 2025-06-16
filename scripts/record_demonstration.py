import numpy as np
import os
import time
import click
import shutil
from pynput.keyboard import KeyCode
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop
from lerobot.common.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig
from lerobot.common.utils.robot_utils import busy_wait

from piper.teleop.robots.robot import Robot
from piper.teleop.robots.visualizer import RobotVisualizer
from piper.teleop.pre_processor.mouse_piper import MousePiperPreProcessor
from piper.teleop.solvers.ik_solver import IKSolver
from piper.teleop.device.mouse import MouseReader
from piper.teleop.device.iphone import IphoneReader
from piper.teleop.pre_processor.iphone_piper import IphonePiperPreProcessor

@click.command()
@click.option("--real", is_flag=True, help="Enable real mode")
@click.option("--fps", type=int, default=10, help="Frame rate")
@click.option("--device", type=click.Choice(['mouse', 'iphone']), default="iphone", help="Device type (mouse or iphone)")
@click.option("--task", type=str, default="test", help="Task name")
@click.option("--debug", is_flag=True, help="Enable debug mode")
def main(real: bool = False, fps: int = 10, device: str = "iphone", task: str = "test", debug: bool = False):
    # Create figure for 3D plotting
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
    last_plot_update = time.time()
    plot_update_interval = 0.05  # 20Hz update rate for plot
    
    def update_plot():
        nonlocal last_plot_update
        current_time = time.time()
        if current_time - last_plot_update >= plot_update_interval:
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
                last_plot_update = current_time

    # enable real mode
    enable_real_device = real

    # asset path
    asset_path = os.path.join(os.path.dirname(__file__), "..", "src/piper/teleop/asset/")
    urdf_path = os.path.join(asset_path, "piper_description/urdf/piper_description.urdf")

    # robot
    sim_robot = Robot("piper", urdf_path=urdf_path, asset_path=asset_path, auto_clip=True)

    # visualizer
    visualizer = RobotVisualizer(sim_robot)
    visualizer.visualize(sim_robot.configuration.q)
    time.sleep(1)

    # preprocessor: transform the device data -> robot's transformation
    pre_processor = MousePiperPreProcessor() if device == "mouse" else IphonePiperPreProcessor()
    pre_processor.register(sim_robot)

    ## postprocessor: simulation robot's action -> real robot's motor command
    # post_processor = SO100PostProcessor()

    # solver: solve IK
    solver = IKSolver(link_costs={
        "gripper_base": {
        "position_cost": 1.0,
        "orientation_cost": 1.0, # 0.0 means no rotation constraint
        }
    })
     
    # register solver to robot
    sim_robot.solver = solver
    solver.register_robot(sim_robot)

    # real robot
    if enable_real_device:
        pass # not implemented

    # Main Loop
    keyboard_teleop = KeyboardTeleop(KeyboardTeleopConfig())
    keyboard_teleop.connect() 

    ControlDevice = MouseReader if device == "mouse" else IphoneReader
    with ControlDevice() as control_device:
        # calibrate
        pre_processor.calibrate()

        # flow control
        stop = False
        is_recording = False
        is_resetting = False
        resetting_count_down = 0.0
        recorded_episodes = 0

        while not stop:
            start_loop_t = time.perf_counter()

            # get action from keyboard
            # action = keyboard_teleop.get_action()
            # TODO: implement flow control

            # ik
            gripper_command = None
            if not is_resetting:
                if device == "iphone":
                    data = control_device.read()[0] 
                    current_position = data[:3, 3]
                elif device == "mouse":
                    data = control_device.read()[0]
                    current_position = data[:3]
                    button = control_device.read()[1]
                    gripper_command = "open" if button[0] == 1 else "close" if button[1] == 1 else "idle"
                
                print("!!!!!!!!!!!!!!!!!!!!!!data", data)
                print("!!!!!!!!!!!!!!!!!!!!!!gripper_command", gripper_command)
                position_history.append(current_position.tolist())  # Add to position history
                # update_plot()  # Update the plot with new position
                

                target = pre_processor(data)
                # print("target", target)
        
                qt = sim_robot.inverse_kinematics(target, gripper_command)

            # visualize
            visualizer.visualize(qt)

            # wait for the control loop to run at the desired frame rate
            dt_s = time.perf_counter() - start_loop_t
            wait_time = 1 / fps - dt_s
            if wait_time < 0:
                print(f"Warning: loop took {dt_s} seconds, which is longer than the desired frame rate of {fps} fps.")
            busy_wait(wait_time)

if __name__ == "__main__":
    main()