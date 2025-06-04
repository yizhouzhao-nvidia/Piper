from piper_dev.controller import BaseController
from typing import Optional, Callable, Dict

class PiperRobot:
    def __init__(self,
                 arm_controller: Optional[BaseController] = None,
                 gripper_controller: Optional[BaseController] = None,
                 no_real: bool = False,
                 ):
        self.arm_controller = arm_controller
        self.gripper_controller = gripper_controller
        self.no_real = no_real

    def set_no_real(self):
        self.no_real = True
