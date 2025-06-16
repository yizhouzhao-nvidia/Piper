from .base_preprocessor import PreProcessor
from scipy.spatial.transform import Rotation as R
import numpy as np
from copy import deepcopy

class MousePiperPreProcessor(PreProcessor):
    def __init__(self, move_scale = 0.1, rotate_scale = 50, ee_name: str = "gripper_base"):
        super().__init__()
        self.pose = None
        self.rotate_scale = rotate_scale
        self.move_scale = move_scale
        self.ee_name = ee_name
        self.ee_pose = None

    def calibrate(self):
        """input the current """
        q = deepcopy(self.robot.robot.q0)

        q[1] = np.deg2rad(90)
        q[2] = np.deg2rad(-45)
        q[4] = np.deg2rad(-40)
        self.robot.update(q)
        self.ee_pose = self.robot.get_link_transformations([self.ee_name])[0]
        self.robot.reset()

    def __call__(self, data: np.ndarray | list) -> dict:
        target_pose = np.eye(4)
        target_pose[0, 3] = self.ee_pose[0, 3] + data[1] * self.move_scale
        target_pose[1, 3] = self.ee_pose[1, 3] + data[0] * self.move_scale
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
        