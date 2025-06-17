import logging
logging.getLogger().setLevel(logging.INFO)  # or WARNING to be stricter

import numpy as np
import os
import time
import click
from multiprocessing.managers import SharedMemoryManager

from piper.real.control_client.piper_client import PiperClient
from piper.real.controller.interpolation_controller import InterpolationController
from piper.real.embodiment.base_embodiment import Embodiment
from piper.real.embodiment.piper import Piper

from lerobot.common.utils.robot_utils import busy_wait

from piper.teleop.robots.robot import Robot
from piper.teleop.robots.visualizer import RobotVisualizer
from piper.teleop.pre_processor.mouse_piper import MousePiperPreProcessor
from piper.teleop.solvers.ik_solver import IKSolver
from piper.teleop.device.mouse import MouseReader
from piper.teleop.device.iphone import IphoneReader
from piper.teleop.pre_processor.iphone_piper import IphonePiperPreProcessor
from piper.teleop.post_processor.piper import PiperPostProcessor

@click.command()
@click.option("--real", is_flag=True, help="Enable real mode")
@click.option("--fps", type=int, default=10, help="Frame rate")
@click.option("--device", type=click.Choice(['mouse', 'iphone']), default="mouse", help="Device type (mouse or iphone)")
def main(real: bool = False, fps: int = 10, device: str = "mouse"):
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
    pre_processor = MousePiperPreProcessor() if device == "mouse" else IphonePiperPreProcessor()
    pre_processor.register(sim_robot)

    ## postprocessor: simulation robot's action -> real robot's motor command
    post_processor = PiperPostProcessor()

    # solver: solve IK
    solver = IKSolver(link_costs={
        "gripper_base": {
        "position_cost": 1.0,
        "orientation_cost": 1.0, # 0.0 means no rotation constraint
        }
    })
     
    # register solver to robot
    sim_robot.solver = solver
    solver.register_robot(sim_robot)

    # real robot
    if enable_real_device:
        pass # not implemented


    with SharedMemoryManager() as shm_manager:
        controller = InterpolationController(shm_manager, PiperClient(), frequency=fps, verbose=True)
        embodiment = Piper(controller) if enable_real_device else Embodiment()

        with embodiment.activate():
            ControlDevice = MouseReader if device == "mouse" else IphoneReader
            with ControlDevice() as control_device:
                # calibrate
                pre_processor.calibrate()

                # go to initial pose
                if enable_real_device:
                    init_pose = np.array([0, np.deg2rad(90), np.deg2rad(-45), 0, np.deg2rad(-40), 0, 0])
                    embodiment.act({'target_pose': init_pose, 'target_time': 4.0 + time.time()})
                    time.sleep(4.0)
                    # init_data = control_device.read()[0]
                    # init_target = pre_processor(init_data)
                    # for _ in range(5 * fps):
                    #     init_qt = sim_robot.inverse_kinematics(init_target, "open")
                    #     visualizer.visualize(init_qt)
                    #     time.sleep(1 / fps)

                # flow control
                t_start = time.monotonic()
                stop = False
                is_recording = False
                is_resetting = False
                resetting_count_down = 0.0
                recorded_episodes = 0
                iter_idx = 0
                dt = 1.0 / fps

                while not stop:

                    t_cycle_end = t_start + (iter_idx + 1) * dt
                    import ipdb; ipdb.set_trace()
                    t_command_target = t_cycle_end + dt


                    start_loop_t = time.perf_counter()

                    # get action from keyboard
                    # action = keyboard_teleop.get_action()
                    # TODO: implement flow control

                    # ik
                    gripper_command = None
                    if not is_resetting:
                        if device == "iphone":
                            data = control_device.read()[0] 
                            current_position = data[:3, 3]
                        elif device == "mouse":
                            data = control_device.read()[0]
                            current_position = data[:3]
                            button = control_device.read()[1]
                            gripper_command = "open" if button[0] == 1 else "close" if button[1] == 1 else "idle"
                        

                        target = pre_processor(data)
                        # print("target", target)
                
                        qt = sim_robot.inverse_kinematics(target, gripper_command)
                        target_cmd = post_processor(qt)

                        # print and keep 2 decimal places
                        print("target_cmd", np.round(target_cmd, 2))
                        print("need time", t_command_target-time.monotonic())
                        embodiment.act({'target_pose': target_cmd, 'target_time': t_command_target-time.monotonic()+time.time()})

                    # visualize
                    visualizer.visualize(qt)

                    # wait for the control loop to run at the desired frame rate
                    dt_s = time.perf_counter() - start_loop_t
                    wait_time = 1.0 / fps - dt_s
                    print("wait time", wait_time)
                    if wait_time < 0:
                        logging.warning(f"Warning: loop took {dt_s} seconds, which is longer than the desired frame rate of {fps} fps.")
                    busy_wait(wait_time)
                    iter_idx += 1

if __name__ == "__main__":
    main()