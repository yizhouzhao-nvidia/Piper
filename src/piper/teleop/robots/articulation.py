import logging
import os
from copy import deepcopy
from typing import Dict, List

import numpy as np
import pink
import pinocchio as pin
from pink.configuration import get_root_joint_dim

logging.basicConfig(format="%(levelname)s:%(message)s", level=logging.DEBUG)


def get_root_dir():
    """Get the root directory of the project."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Articulation:
    def __init__(
        self,
        name: str,
        urdf_path: str,
        asset_path: str,
        auto_clip: bool = False,
        **kwargs,
    ) -> None:

        self.name = name

        # Get the root directory of the project
        root_dir = get_root_dir()
        self.urdf_path = os.path.join(root_dir, urdf_path)
        self.asset_path = os.path.join(root_dir, asset_path)
        self.auto_clip = auto_clip

        # Load the URDF file
        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=self.urdf_path,
            package_dirs=[self.asset_path],
            root_joint=None,
        )
        print("[Robot model]: ", self.robot.model)
        self.configuration = pink.Configuration(self.robot.model, self.robot.data, self.robot.q0)
        self.initial_upper = deepcopy(self.robot.model.upperPositionLimit)
        self.initial_lower = deepcopy(self.robot.model.lowerPositionLimit)

        self.joint2idx = {k: self.robot.model.getJointId(k) - 1 for k in self.robot.model.names[1:]}
        self.actuator_index_list = []
        self.joints_to_lock_ids = []

        if "joint_limits" in kwargs:
            self.set_joint_limits(kwargs["joint_limits"])

    def set_joint_limits(self, joint_limits: Dict[str, List[float]]) -> None:
        """Set the joint limits of the robot."""
        # print("[Setting joint limits]: ", joint_limits)
        for joint_name, limits in joint_limits.items():
            if joint_name in self.joint2idx:
                joint_id = self.joint2idx[joint_name]
                self.configuration.model.lowerPositionLimit[joint_id] = limits[0]
                self.configuration.model.upperPositionLimit[joint_id] = limits[1]

    @property
    def num_dof(self) -> int:
        """Get the number of degrees of freedom of the robot."""
        return self.robot.q0.shape[0]

    @property
    def dof_names(self) -> List[str]:
        """Get the names of the degrees of freedom of the robot."""
        return list(self.joint2idx.keys())

    def get_joint_positions(self) -> np.ndarray:
        """Get the joint positions of the robot."""
        q = self.configuration.q.copy()
        return q

    def get_link_transformations(self, link_names) -> List[np.ndarray]:
        return [
            self.configuration.get_transform_frame_to_world(link_name).np
            for link_name in link_names
        ]

    def update(self, q, tol=1e-6):
        q = deepcopy(q)
        if self.auto_clip:
            root_nq, _ = get_root_joint_dim(self.configuration.model)
            q_max = self.configuration.model.upperPositionLimit[root_nq:]
            q_min = self.configuration.model.lowerPositionLimit[root_nq:]
            q[root_nq:] = np.clip(q[root_nq:], q_min + tol, q_max - tol)

        self.configuration.q = q
        self.configuration.update()

    def reset(self):
        self.update(self.robot.q0)