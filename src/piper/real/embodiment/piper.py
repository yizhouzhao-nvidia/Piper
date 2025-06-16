from piper.real.controller import BaseController
from typing import Optional, Callable, Dict, List, Union
from contextlib import contextmanager
import numpy as np
import time

class Piper:
    def __init__(self,
                 piper_controller: Optional[BaseController] = None,
                 no_real: bool = False,
                 ):
        self.piper_controller = piper_controller
        self.no_real = no_real

    def set_no_real(self):
        self.no_real = True

    def act(self, action: Dict[str, Union[np.ndarray, List[float], float]]):
        if self.no_real:
            return
        
        target_pose = action['target_pose']
        target_time = action['target_time']

        self.piper_controller.schedule_waypoint(np.array(target_pose), target_time)
        
    def go_to_zero_pose(self):
        self.piper_controller.schedule_waypoint(np.zeros(7), 3.0 + time.time())
        
    @contextmanager
    def activate(self):
        try:
            piper_controller = self.piper_controller if self.piper_controller is not None else BaseController()
            with piper_controller.activate():
                self.go_to_zero_pose()
                yield self
        finally:
            print("[Embodiment] piper finished")
            self.piper_controller = None
