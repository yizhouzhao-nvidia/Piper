from typing import Dict

import numpy as np
import pinocchio as pin
import qpsolvers
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask

from .basic_solver import Solver


class WeightedPostureTask(PostureTask):
    def __init__(
        self, cost: float, weights: np.ndarray, lm_damping: float = 0.0, gain: float = 1.0
    ) -> None:
        r"""Create weighted posture task.

        Args:
            cost: value used to cast joint angle differences to a homogeneous
                cost, in :math:`[\mathrm{cost}] / [\mathrm{rad}]`.
            weights: vector of weights for each joint.
            lm_damping: Unitless scale of the Levenberg-Marquardt (only when
                the error is large) regularization term, which helps when
                targets are unfeasible. Increase this value if the task is too
                jerky under unfeasible targets, but beware that too large a
                damping can slow down the task.
            gain: Task gain :math:`\alpha \in [0, 1]` for additional low-pass
                filtering. Defaults to 1.0 (no filtering) for dead-beat
                control.
        """
        super().__init__(cost=cost, lm_damping=lm_damping, gain=gain)
        self.weights = weights

    def compute_error(self, configuration):
        error = super().compute_error(configuration)
        return self.weights * error

    def compute_jacobian(self, configuration):
        J = super().compute_jacobian(configuration)
        # breakpoint()
        return self.weights[:, np.newaxis] * J

    def __repr__(self):
        """Human-readable representation of the weighted posture task."""
        return (
            "WeightedPostureTask("
            f"cost={self.cost}, "
            f"weights={self.weights}, "
            f"gain={self.gain}, "
            f"lm_damping={self.lm_damping})"
        )


class IKSolver(Solver):
    def __init__(
        self,
        link_costs: Dict,
        posture_weight: Dict = None,
        dt: float = 0.01,
        num_step_per_frame: int = 1,
        posture_cost: float = 0.01,
        posture_lm_damping: float = 1.0,
        amplify_factor: float = 1.0,
    ):
        # load robot
        # super().__init__({}, robot)
        self.dt = dt
        self.num_step_per_frame = num_step_per_frame
        self.amplify_factor = amplify_factor
        self.link_costs = link_costs
        self.posture_weight = posture_weight
        self.posture_cost = posture_cost
        self.posture_lm_damping = posture_lm_damping
        self.robot = None

    def register_robot(self, robot):
        self.robot = robot
        self.initialize()

    def initialize(self):
        self.solver = qpsolvers.available_solvers[0]
        if "quadprog" in qpsolvers.available_solvers:
            self.solver = "quadprog"
        else:
            self.solver = qpsolvers.available_solvers[0]

        # initialize tasks
        self.tasks = {}
        for link_name, weight in self.link_costs.items():
            assert link_name != "posture", "posture is a reserved task name"
            task = FrameTask(
                link_name,
                **weight,
            )
            self.tasks[link_name] = task

        # add posture task
        if self.posture_weight is not None:
            # breakpoint()
            weight = np.ones(self.robot.num_dof)
            for joint_name, posture_weight in self.posture_weight.items():
                joint_idx = self.robot.joint2idx[joint_name]
                weight[joint_idx] = posture_weight

            self.tasks["posture"] = WeightedPostureTask(
                cost=self.posture_cost,
                weights=weight,
                lm_damping=self.posture_lm_damping,
            )
        else:
            self.tasks["posture"] = PostureTask(
                cost=self.posture_cost, lm_damping=self.posture_lm_damping
            )
        for task in self.tasks.values():
            task.set_target_from_configuration(self.robot.configuration)

    def __call__(self, target_pose: Dict):
        for link_name, pose in target_pose.items():
            if link_name not in self.tasks:
                continue
            pose = pin.SE3(pose[:3, :3], pose[:3, 3])
            self.tasks[link_name].set_target(pose)
        # for link_name in self.tasks.keys():
        #     if link_name == "posture":
        #         continue
        #     if link_name not in target_pose:
        #         continue
        #     target_transform = target_pose[link_name]
        #     target_transform = pin.SE3(target_transform[:3, :3], target_transform[:3, 3])
        #     self.tasks[link_name].set_target(target_transform)

        for _ in range(self.num_step_per_frame):
            velocity = solve_ik(
                self.robot.configuration,
                self.tasks.values(),
                dt=self.dt,
                solver=self.solver,
            )
            self.robot.update(
                self.robot.configuration.q + velocity * self.dt * self.amplify_factor,
                tol=1e-6,
            )

        return self.robot.configuration.q.copy()
        # return self.robot.configuration.q + velocity * self.dt * self.amplify_factor

    def update_weights(self, weights):
        for link_name, weight in weights.items():
            if "position_cost" in weight:
                self.tasks[link_name].set_position_cost(weight["position_cost"])
            if "orientation_cost" in weight:
                self.tasks[link_name].set_orientation_cost(weight["orientation_cost"])