from typing import Dict, List, Union

import numpy as np

from piper.teleop.solvers.basic_solver import Solver
from .articulation import Articulation


class Robot(Articulation):
    def __init__(
        self,
        name: str,
        urdf_path: str,
        asset_path: str,
        auto_clip: bool,
        solver: Union[Solver, None] = None,
        **kwargs,
    ) -> None:
        Articulation.__init__(
            self, name, urdf_path, asset_path, auto_clip, **kwargs
        )
        
        # gripper joint index
        self.gripper_joint_idx = -1
        
        # register solver target
        self.solver = solver
        if self.solver is not None:
            self.solver.register_robot(self)

    def forward_kinematics(
        self, q: List[float], link_names: List[str], from_real_actuators=True
    ) -> List[np.ndarray]:
        """
        Get the end-effector pose of the robot for the given configuration.
        Args:
            q: Configuration vector.
            link_names: List of link names to get the end-effector pose.
        Returns:
            Dictionary of link names and their corresponding end-effector pose.
        """
        self.update(q)
        return self.get_link_transformations(link_names)

    def inverse_kinematics(
        self, target_pose: Dict[str, np.ndarray], gripper_command = "open"
    ) -> List[float]:
        """
        Solve the inverse kinematics problem for the given target poses.
        Args:
            target_pose: Dictionary of link names and their corresponding target pose.
            q: Initial configuration vector.
        Returns:
            Configuration vector that achieves the target poses.
        """
        current_gripper_joint_q = self.get_joint_positions()[-1]
        if self.solver is None:
            raise ValueError("Solver is not initialized.")

        qt = self.solver(target_pose)
        # if gripper_command == "open":
        #     q[-1] = np.clip(current_gripper_joint_q + 0.05, 0, np.pi / 2)
        # elif gripper_command == "close":
        #     q[-1] = np.clip(current_gripper_joint_q - 0.05, 0, np.pi / 2)
        # else:
        #     q[-1] = current_gripper_joint_q
        

        # import ipdb; ipdb.set_trace()
        self.update(qt)
        
        # if include_fixed is True, add the fixed joints to the configuration
        return self.get_joint_positions()