from .base_postprocessor import PostProcessor
import torch
from typing import List
import numpy as np

class PiperPostProcessor(PostProcessor):
    def __init__(self):
        super().__init__()
        
    def __call__(self, data: List[float]):
        # map the gripper command 0 to 0.05 to 0 to 8000
        gripper_command = abs(data[-1])
        gripper_command = int(gripper_command * 1000)

        target_cmd = np.zeros(7)
        target_cmd[0] = data[0]
        target_cmd[1] = data[1]
        target_cmd[2] = data[2]
        target_cmd[3] = data[3]
        target_cmd[4] = data[4]
        target_cmd[5] = data[5]
        target_cmd[6] = gripper_command
        
        return target_cmd
        