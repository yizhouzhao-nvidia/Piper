import numpy as np
import os
import time
import click
import shutil
from pynput.keyboard import KeyCode

from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop
from lerobot.common.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig
from lerobot.common.utils.robot_utils import busy_wait

from piper.teleop.robots.robot import Robot
from piper.teleop.robots.visualizer import RobotVisualizer
from piper.teleop.pre_processor.mouse_so100 import MousePiperPreProcessor
from piper.teleop.solvers.ik_solver import IKSolver
from piper.teleop.device.mouse import MouseReader

@click.command()
@click.option("--real", is_flag=True, help="Enable real mode")
@click.option("--fps", type=int, default=20, help="Frame rate")
@click.option("--device", type=str, default="mouse", help="Device type")
@click.option("--task", type=str, default="test", help="Task name")
@click.option("--debug", is_flag=True, help="Enable debug mode")
def main(real: bool = True, fps: int = 20, device: str = "mouse", task: str = "test", debug: bool = False):
    # enable real mode
    enable_real_device = real

    # asset path
    asset_path = os.path.join(os.path.dirname(__file__), "..", "src/piper/teleop/asset/")
    urdf_path = os.path.join(asset_path, "piper_description/urdf/piper_description.urdf")

    # robot
    sim_robot = Robot("piper", urdf_path=urdf_path, asset_path=asset_path, auto_clip=True)

    # visualizer
    visualizer = RobotVisualizer(sim_robot)
    visualizer.visualize(sim_robot.configuration.q)
    time.sleep(1)

    # preprocessor: transform the device data -> robot's transformation
    pre_processor = MousePiperPreProcessor()
    pre_processor.register(sim_robot)

    ## postprocessor: simulation robot's action -> real robot's motor command
    # post_processor = SO100PostProcessor()

    # solver: solve IK
    solver = IKSolver(link_costs={
        "gripper_base": {
        "position_cost": 1.0,
        "orientation_cost": 0.5, # 0.0 means no rotation constraint
        }
    })
     
    # register solver to robot
    sim_robot.solver = solver
    solver.register_robot(sim_robot)

    # real robot
    if enable_real_device:
        pass # not implemented

    # Main Loop
    keyboard_teleop = KeyboardTeleop(KeyboardTeleopConfig())
    keyboard_teleop.connect() 

    with MouseReader() as control_device:
        # calibrate
        pre_processor.calibrate()

        # flow control
        stop = False
        is_recording = False
        is_resetting = False
        resetting_count_down = 0.0
        recorded_episodes = 0

        while not stop:
            start_loop_t = time.perf_counter()

            # get action from keyboard
            action = keyboard_teleop.get_action()
            # TODO: implement flow control

            # ik
            if not is_resetting:
                data = control_device.read()[0] 
                # print("data", data)
                button = control_device.read()[1]
                gripper_command = "open" if button[0] == 1 else "close" if button[1] == 1 else "idle"
                target = pre_processor(data)
                # print("target", target)
                qt = sim_robot.inverse_kinematics(target, gripper_command)

            # visualize
            visualizer.visualize(qt)


            # wait for the control loop to run at the desired frame rate
            dt_s = time.perf_counter() - start_loop_t
            wait_time = 1 / fps - dt_s
            if wait_time < 0:
                print(f"Warning: loop took {dt_s} seconds, which is longer than the desired frame rate of {fps} fps.")
            busy_wait(wait_time)

if __name__ == "__main__":
    main()