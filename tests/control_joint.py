import time
import os
from multiprocessing.managers import SharedMemoryManager
import numpy as np

from piper.real.control_client.piper_client import PiperClient
from piper.real.controller.interpolation_controller import InterpolationController
from piper.real.embodiment.piper import Piper

from piper.real.utils.precise_sleep import precise_wait

from pynput.mouse import Listener
import click

joint_state = 0

@click.command()
@click.option('--joint_idx', type=int, default=5, help='joint index to control')
def main(joint_idx):
    frequency = 50
    dt = 1. / frequency
    motion_scale = 0.01 # rad
    
    def on_scroll(x, y, dx, dy):
        global joint_state
        if dy != 0:
            if dy > 0:
                # print("scroll up")
                joint_state += motion_scale
            else:
                # print("scroll down")
                joint_state -= motion_scale
        
            print(f"joint_state: {joint_state}")
            target_pose = np.zeros(7)
            target_pose[joint_idx] = joint_state
            embodiment.act({'target_pose': target_pose, 'target_time': 0.1 + time.time()})
            
    
    with SharedMemoryManager() as shm_manager:
        controller = InterpolationController(shm_manager, PiperClient(), frequency=frequency, verbose=False)
        embodiment = Piper(controller)

        with embodiment.activate():
            with Listener(on_scroll=on_scroll) as listener:
                listener.join()



if __name__ == "__main__":
    main()



