import enum
import multiprocessing as mp
import os
import time
from contextlib import contextmanager
from multiprocessing.managers import SharedMemoryManager

import numpy as np

from piper_dev.control_client import BaseClient
from piper_dev.shared_memory.shared_memory_queue import Empty, SharedMemoryQueue
from piper_dev.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from piper_dev.utils.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from piper_dev.utils.precise_sleep import precise_wait

from .base_controller import BaseController

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2


class InterpolationController(mp.Process, BaseController):
    def __init__(self,
            shm_manager: SharedMemoryManager,
            control_client: BaseClient,
            frequency=400,
            launch_timeout=3,
            get_max_k=20,
            max_joint_speed=np.inf,
            soft_real_time=False,
            verbose=False,
        ):
        super().__init__(name="InterpolationController")

        self.control_client = control_client
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.max_joint_speed = max_joint_speed
        self.soft_real_time = soft_real_time
        self.verbose = verbose

        # build input queue
        input_example = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': np.zeros(self.control_client.cmd_shape, dtype=np.float64),
            'duration': 0.0,
            'target_time': 0.0
        }
        self.input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=input_example,
            buffer_size=256
        )

        output_example = {
            'state': np.zeros(self.control_client.state_shape, dtype=np.float64),
            'timestamp': time.time(),
        }
        self.output_ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=output_example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        
        if self.verbose:
            print(f"[InterpolationController] Controller initialized")

    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[InterpolationController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.STOP.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()

    def stop_wait(self):
        self.join()
    
    @property
    def is_ready(self):
        return self.ready_event.is_set()
    
    # ========= context manager ===========
    @contextmanager
    def activate(self):
        try:
            self.start()
            if self.verbose:
                print(f"[InterpolationController] Controller process started")
            yield self
        finally:
            self.stop()
            if self.verbose:
                print(f"[InterpolationController] Controller process stopped")

    # ========= command methods ============
    def schedule_waypoint(self, target_pose, target_time):
        assert target_pose.shape == self.control_client.cmd_shape

        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': target_pose,
            'target_time': target_time
        }
        self.input_queue.put(message)
    
    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.output_ring_buffer.get(out=out)
        else:
            return self.output_ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.output_ring_buffer.get_all()

    # ========= main loop in process ============
    def run(self):
        if self.verbose:
            print(f"[InterpolationController] Running")
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))
        
        # data_log = list()
        with self.control_client.activate():
            # main loop
            dt = 1. / self.frequency
            curr_pose = self.control_client.get()
            
            if self.verbose:
                print(f"[InterpolationController] Control client activated")
                print(f"[InterpolationController] Current pose: {curr_pose}")
                            
            # use monotonic time to make sure the control loop never go backward
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[curr_pose]
            )

            t_start = time.monotonic()
            iter_idx = 0
            keep_running = True
            
            while keep_running:
                # send command to robot
                t_now = time.monotonic()

                pose_command = pose_interp(t_now)
                
                if self.verbose:
                    print(f"[InterpolationController] Pose command: {pose_command}")
                
                self.control_client.move(pose_command)

                state = {
                    'state': self.control_client.get(),
                    'timestamp': time.time(),
                }

                self.output_ring_buffer.put(state)

                # fetch command from queue
                try:
                    # commands = self.input_queue.get_all()
                    # n_cmd = len(commands['cmd'])
                    # process at most 1 command per cycle to maintain frequency
                    # commands = self.input_queue.get_k(1)
                    # commands = self.input_queue.get_all()
                    commands = self.input_queue.get_k(3, exact=False)
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']

                    if cmd == Command.STOP.value:
                        keep_running = False
                        break
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']
                        target_time = float(command['target_time'])
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            max_joint_speed=self.max_joint_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                # regulate frequency
                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[InterpolationController] Actual frequency {1/(time.monotonic() - t_now)}")

        self.ready_event.set()
        if self.verbose:
            print(f"[InterpolationController] Disconnected from robot")


class CompositeInterpolationController:
    def __init__(self, *args):
        self.controllers = args
        self.num_controller = len(args)
        self.no_real = False
    
    def __enter__(self):
        for controller in self.controllers:
            controller.__enter__()
        return self
    
    @contextmanager
    def activate(self):
        try:
            for controller in self.controllers:
                controller.start()
            yield self
        finally:
            for controller in self.controllers:
                controller.stop()

    def schedule_waypoint(self, target_poses, target_times):
        if self.no_real:
            return
        if isinstance(target_times, (int, float)):
            target_times = [target_times] * self.num_controller
        
        for idx in range(self.num_controller):
            self.controllers[idx].schedule_waypoint(target_poses[idx], target_times[idx])

    def get_state(self, k=None, out=None):
        return [controller.get_state(k, out) for controller in self.controllers]
    
    def get_all_state(self):
        return [controller.get_all_state() for controller in self.controllers]
    
    def set_no_real(self):
        self.no_real = True