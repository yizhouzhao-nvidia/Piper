from .base_postprocessor import PostProcessor
import torch
from typing import List
import numpy as np

class SO100PostProcessor(PostProcessor):
    def __init__(self, joint_offset = [90., 90., 90., 90., 90.,  0.]):
        super().__init__()
        self.join_offset = np.asarray(joint_offset)
        
    def __call__(self, data: List[float]):
        data[4] = -data[4]
        # data[3] = -data[3]
        # data[2] = -data[2]
        data[1] = -data[1]
        data[0] = data[0]
        data = np.rad2deg(data) + self.join_offset
        return torch.tensor(data).float()
        