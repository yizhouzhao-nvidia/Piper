from contextlib import contextmanager
import time
import numpy as np

from piper_sdk import C_PiperInterface_V2  

from .base_client import BaseClient

def enable_fun(piper:C_PiperInterface_V2, enable:bool):
    '''
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    '''
    enable_flag = False
    loop_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (loop_flag):
        elapsed_time = time.time() - start_time
        print(f"--------------------")
        enable_list = []
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
        enable_list.append(piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
        if(enable):
            enable_flag = all(enable_list)
            piper.EnableArm(7)
            piper.GripperCtrl(0,1000,0x01, 0)
        else:
            enable_flag = any(enable_list)
            piper.DisableArm(7)
            piper.GripperCtrl(0,1000,0x02, 0)
        print(f"[PiperClient] Enable state: {enable_flag}")
        print(f"--------------------")
        if(enable_flag == enable):
            loop_flag = True
            enable_flag = True
        else: 
            loop_flag = False
            enable_flag = False
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print(f"[PiperClient] Timeout....")
            elapsed_time_flag = True
            enable_flag = False
            loop_flag = True
            break
        time.sleep(0.5)
    resp = enable_flag
    print(f"[PiperClient] Returning response: {resp}")
    return resp


class PiperClient(BaseClient):
    def __init__(self,
                 port: str = "can0"):
        super().__init__()
        self.port = port
        self.piper = None
        
        self.joint_limit_rad = np.array([
            [-2.618, 2.618],
            [0, 3.14],
            [-2.967, 0],
            [-1.745, 1.745],
            [-1.22, 1.22],
            [-2.0944, 2.0944],
            [0, 100],
        ])


    @contextmanager
    def activate(self):
        """
        Activate the Piper SDK.
        """
        try:
            self.piper = C_PiperInterface_V2(self.port)
            self.piper.ConnectPort(True)
            if self.piper:
                time.sleep(0.025) # 需要时间去读取固件反馈帧，否则会反馈-0x4AF
                version = self.piper.GetPiperFirmwareVersion()
                print(f"Connected to Piper with SDK version: {version}")
                flag = enable_fun(piper=self.piper, enable=True)
                if(flag == True):
                    print("[PiperClient] Enable Successful !!!!")
                else:
                    raise Exception("Failed to enable Piper")
                
                # Go to Zero Position             
                
                self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00) # CAN mode, Joint control, speed percent, mit mode (position, speed)
                self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
                # self.piper.GripperCtrl(0, 1000, 0x01, 0) 
                self.piper.GripperCtrl(round(0.02 * 1e6), 1000, 0x01, 0)
                
            else:
                raise Exception("Failed to connect to Piper")
            yield self.piper
        
        except Exception as e:
            print(f"[Client] Error: {e}")
        
        finally:
            if self.piper:    
                # time.sleep(2)
                self.piper.MotionCtrl_1(0x01,0,0) # stop
                for i in range(5):
                    print(f"[Info] Stopping Piper....count down {5-i} ...")
                    time.sleep(1)
                
                self.piper.MotionCtrl_1(0x02, 0x00, 0x00) # reset
                print("[Info] Resetting Piper....")
                
                flag = enable_fun(piper=self.piper, enable=False)
                if(flag == True):
                    print("[PiperClient] Disable Successful !!!!")

        
                self.piper.DisconnectPort()
                self.piper = None
                
    @property
    def cmd_shape(self):
        return (6 + 1, )

    @property
    def state_shape(self):
        return (6 + 1, ) 
    
    def move(self, cmds: np.ndarray):
        # clip the cmds according to the joint limit:
        #         |joint_name|     limit(rad)     |    limit(angle)    |
        # |----------|     ----------     |     ----------     |
        # |joint1    |   [-2.618, 2.618]  |    [-150.0, 150.0] |
        # |joint2    |   [0, 3.14]        |    [0, 180.0]      |
        # |joint3    |   [-2.967, 0]      |    [-170, 0]       |
        # |joint4    |   [-1.745, 1.745]  |    [-100.0, 100.0] |
        # |joint5    |   [-1.22, 1.22]    |    [-70.0, 70.0]   |
        # |joint6    |   [-2.0944, 2.0944]|    [-120.0, 120.0] |    
        
        cmds = np.clip(cmds, self.joint_limit_rad[:, 0], self.joint_limit_rad[:, 1])
        
        factor = 57295.7795 #1000*180/3.1415926
        joint_0 = round(cmds[0]*factor)
        joint_1 = round(cmds[1]*factor)
        joint_2 = round(cmds[2]*factor)
        joint_3 = round(cmds[3]*factor)
        joint_4 = round(cmds[4]*factor)
        joint_5 = round(cmds[5]*factor)
        joint_6 = round(cmds[6]*1000)
        
        
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00) # CAN mode, Joint control, speed percent, mit mode (position, speed)
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        
    def get(self):
        arm_joint_msg = self.piper.GetArmJointMsgs()
        gripper_msg = self.piper.GetArmGripperMsgs()
        arm_joint_state = [np.deg2rad(arm_joint_msg.joint_state.joint_1 * 0.001), 
                           np.deg2rad(arm_joint_msg.joint_state.joint_2 * 0.001), 
                           np.deg2rad(arm_joint_msg.joint_state.joint_3 * 0.001), 
                           np.deg2rad(arm_joint_msg.joint_state.joint_4 * 0.001), 
                           np.deg2rad(arm_joint_msg.joint_state.joint_5 * 0.001), 
                           np.deg2rad(arm_joint_msg.joint_state.joint_6 * 0.001)] + \
                         [gripper_msg.gripper_state.grippers_angle * 0.001] # in mm   
                         
        return np.array(arm_joint_state)
        
        
        