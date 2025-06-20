import time
import os
from multiprocessing.managers import SharedMemoryManager
import numpy as np

from piper.real.control_client.piper_client import PiperClient
from piper.real.controller.interpolation_controller import InterpolationController
from piper.real.embodiment.piper import Piper

from piper.real.utils.precise_sleep import precise_wait

from pynput.mouse import Listener

gripper_state = 0

def main():
    frequency = 50
    dt = 1. / frequency
    motion_scale = 2 # 20 mm
    
    def on_scroll(x, y, dx, dy):
        global gripper_state
        if dy != 0:
            if dy > 0:
                # print("scroll up")
                gripper_state += motion_scale
            else:
                # print("scroll down")
                gripper_state -= motion_scale
        
            print(f"gripper_state: {gripper_state}")
            target_pose = np.zeros(7)
            target_pose[6] = gripper_state
            embodiment.act({'target_pose': target_pose, 'target_time': 0.1 + time.time()})
            
    
    with SharedMemoryManager() as shm_manager:
        controller = InterpolationController(shm_manager, PiperClient(), frequency=frequency, verbose=False)
        embodiment = Piper(controller)

        with embodiment.activate():
            with Listener(on_scroll=on_scroll) as listener:
                listener.join()



if __name__ == "__main__":
    main()



