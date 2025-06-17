from pynput import mouse
from typing import Tuple
import numpy as np

# FIXME: Implement the MouseReader class
class MouseReader:
    def __init__(self):
        self.initial_x = None
        self.initial_y = None
        self.x, self.y, self.z = 0, 0, 0  # Delta positions
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.buttons = [0, 0]  # Stores "open" or "close" action
        self.listener = mouse.Listener(
            on_move=self.on_move,
            on_scroll=self.on_scroll,
            on_click=self.on_click
        )
        self.next_action = "open"
        self.button_state = {  # Track button press states
            "left": False,
            "right": False,
            "middle": False,
            "side1": False,
            "side2": False
        }

    def on_move(self, x, y):
        """Track delta X, Y movements from the initial position."""
        if self.initial_x is None or self.initial_y is None:
            self.initial_x, self.initial_y = x , y  # Set initial position
        
        self.x, self.y = (x - self.initial_x) / 500.0, (y - self.initial_y) / 500.0
        # print(f"Mouse delta: ({self.x}, {self.y})")

    def on_scroll(self, x, y, dx, dy):
        """Track delta scroll (Z-axis)."""
        self.z += dy / 20  # Scroll up/down maps to Z-axis
        # print(f"Scroll delta: Z={self.z}")

    def on_click(self, x, y, button, pressed):
        """Detect side button clicks and map them to actions."""
        if pressed:
            print("button", button)
            if button == mouse.Button.left:  # Side button 1
                self.roll += 0.05
            elif button == mouse.Button.right:  # Side button 2
                self.roll -= 0.05
            elif button == mouse.Button.middle:  # Middle button click
                if self.next_action == "open":
                    self.buttons[0] = 1
                    self.next_action = "close"
                    # print("Action: Open")
                else:
                    self.buttons[1] = 1
                    self.next_action = "open"
                    # print("Action: Close")
            # elif button == mouse.Button.side:
            #     self.pitch += 0.05
            # elif button == mouse.Button.side2:
            #     self.pitch -= 0.05
        else:
            # idle
            if button == mouse.Button.middle:
                self.buttons[0] = 0
                self.buttons[1] = 0 

                
    def read(self) -> Tuple[np.ndarray, list]:
        """Returns the latest action and button state of the SpaceMouse."""
        return np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw]), self.buttons      

    def __enter__(self):
        """Start the mouse listener."""
        print("Starting MouseReader...")
        self.listener.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.listener.stop()
        if exc_type is not None:
            print(f"An exception of type {exc_type} occurred: {exc_val}")
            
