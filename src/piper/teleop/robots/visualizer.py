from typing import Dict
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g

from .robot import Robot

class RobotVisualizer:
    def __init__(self, robot: Robot):
        self.robot = robot.robot
        self.viz = MeshcatVisualizer(
            self.robot.model, self.robot.collision_model, self.robot.visual_model
        )
        try:
            self.viz.initViewer(open=True)
        except ImportError as err:
            print("Error while initializing the viewer. It seems you should install Python meshcat")
            print(err)
            exit(0)

        self.viz.loadViewerModel()
        self.viz.display(self.robot.q0)
        
        # Create an end-effector marker (small sphere)
        self.viz.viewer["ee_marker"].set_object(g.Sphere(0.02), g.MeshLambertMaterial(color=0xFFFF00))
        self.viz.viewer["ee_axis_x"].set_object(g.Box([0.1, 0.005, 0.005]), g.MeshLambertMaterial(color=0xFF0000))
        self.viz.viewer["ee_axis_y"].set_object(g.Box([0.005, 0.1, 0.005]), g.MeshLambertMaterial(color=0x00FF00))
        self.viz.viewer["ee_axis_z"].set_object(g.Box([0.005, 0.005, 0.1]), g.MeshLambertMaterial(color=0x0000FF))
        
    def visualize(self, robot_state: np.ndarray, ee_frame_name: str = "gripper_base"):
        # Visualize robot state
        if robot_state is not None:
            self.viz.display(robot_state)

            # Compute forward kinematics
            pin.forwardKinematics(self.robot.model, self.robot.data, robot_state)
            pin.updateFramePlacements(self.robot.model, self.robot.data)

            # Get end-effector frame ID (update with actual frame name)
            ee_frame_name = ee_frame_name
            ee_frame_id = self.robot.model.getFrameId(ee_frame_name)

            # Get end-effector pose (position & rotation)
            ee_transform = self.robot.data.oMf[ee_frame_id]
            ee_position = ee_transform.translation
            ee_rotation = ee_transform.rotation

            # 🔥 Convert to 4x4 transformation matrix
            ee_pose = np.eye(4)
            ee_pose[:3, :3] = ee_rotation
            ee_pose[:3, 3] = ee_position

            # Update end-effector marker position
            self.viz.viewer["ee_marker"].set_transform(ee_pose)
            self.viz.viewer["ee_axis_x"].set_transform(ee_pose)
            self.viz.viewer["ee_axis_y"].set_transform(ee_pose)
            self.viz.viewer["ee_axis_z"].set_transform(ee_pose)

            # Print the position and rotation
            # print(f"End-Effector Position: {ee_position}")
            # print(f"End-Effector Rotation:\n{ee_rotation}")
