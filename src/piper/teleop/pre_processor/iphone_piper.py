from .base_preprocessor import PreProcessor
from scipy.spatial.transform import Rotation as R
import numpy as np

def swap_yz_axes(matrix):
    # Make a copy to avoid modifying original
    m = matrix.copy()
    
    # Swap rows 1 and 2 (Y and Z rows)
    m[[1, 2], :] = m[[2, 1], :]
    
    # Swap columns 1 and 2 (Y and Z columns)
    m[:, [1, 2]] = m[:, [2, 1]]
    
    return m

class IphonePiperPreProcessor(PreProcessor):
    def __init__(self, move_scale = 1, rotate_scale = 50, ee_name: str = "gripper_base"):
        super().__init__()
        self.pose = None
        self.rotate_scale = rotate_scale
        self.move_scale = move_scale
        self.ee_name = ee_name
        self.init_ee_pose = None

    def calibrate(self):
        """input the current """
        self.init_ee_pose = self.robot.get_link_transformations([self.ee_name])[0]

    def __call__(self, data: np.ndarray) -> dict:
        device_transform = data
        
        device_transform = swap_yz_axes(device_transform)
        
        # inter change the y and z axis of data
        # device_transform[:3, 3] = device_transform[:3, 3][[0, 2, 1]]
        # print("device_transform", np.round(device_transform[:3, 3], 4))
        
        target_pose = np.eye(4)
        target_pose[:3, 3] = self.init_ee_pose[:3, 3] + device_transform[:3, 3] *  self.move_scale
        # print("[processer] target position", np.round(target_pose[:3, 3], 4))
        
        
        additation_rotation = R.from_matrix(device_transform[:3, :3])
        new_rotation = R.from_matrix(self.init_ee_pose[:3, :3]) * additation_rotation
        target_pose[:3, :3] = new_rotation.as_matrix()  
        
        if np.random.rand() < 0.2:
            print("current_rotation", R.from_matrix(self.init_ee_pose[:3, :3]).as_euler("xyz", degrees=True))
            print("additation_rotation", additation_rotation.as_euler("xyz", degrees=True))
            print("\n new_rotation", new_rotation.as_euler("xyz", degrees=True))

        # update ee_pose
        # self.ee_pose = np.copy(target_pose)
        return {self.ee_name: target_pose}
        