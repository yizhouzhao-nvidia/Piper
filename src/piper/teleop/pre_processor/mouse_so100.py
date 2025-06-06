from .base_preprocessor import PreProcessor
from pyspacemouse import SpaceNavigator
from scipy.spatial.transform import Rotation as R
import numpy as np

class MouseSO100PreProcessor(PreProcessor):
    def __init__(self, move_scale = 0.1, rotate_scale = 50):
        super().__init__()
        self.pose = None
        self.rotate_scale = rotate_scale
        self.move_scale = move_scale
        self.ee_name = "Fixed_Jaw"
        self.ee_pose = None

    def calibrate(self, data: SpaceNavigator):
        """input the current """
        self.ee_pose = self.robot.get_link_transformations([self.ee_name])[0]

    def __call__(self, data: np.ndarray | list) -> dict:
        target_pose = np.eye(4)
        target_pose[0, 3] = self.ee_pose[0, 3] - data[0] * self.move_scale
        target_pose[1, 3] = self.ee_pose[1, 3] + data[1] * self.move_scale
        target_pose[2, 3] = self.ee_pose[2, 3] + data[2] * self.move_scale
        
        target_pose[:3, :3] = self.ee_pose[:3, :3]
        
        current_rotation = R.from_matrix(self.ee_pose[:3, :3])
        
        additation_rotation = R.from_euler("xyz", [0, -data[3] * self.rotate_scale, 0], degrees=True)
        new_rotation = current_rotation * additation_rotation
        target_pose[:3, :3] = new_rotation.as_matrix()  
        
        # if np.random.rand() < 0.2:
        #     print("current_rotation", current_rotation.as_euler("xyz", degrees=True))
        #     print("additation_rotation", additation_rotation.as_euler("xyz", degrees=True))
        #     print("new_rotation", new_rotation.as_euler("xyz", degrees=True))

        # update ee_pose
        # self.ee_pose = np.copy(target_pose)
        return {self.ee_name: target_pose}
        