from contextlib import contextmanager
import time

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
        print(f"使能状态: {enable_flag}")
        print(f"--------------------")
        if(enable_flag == enable):
            loop_flag = True
            enable_flag = True
        else: 
            loop_flag = False
            enable_flag = False
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print(f"超时....")
            elapsed_time_flag = True
            enable_flag = False
            loop_flag = True
            break
        time.sleep(0.5)
    resp = enable_flag
    print(f"Returning response: {resp}")
    return resp

class PiperClient(BaseClient):
    def __init__(self,
                 port: str = "can0"):
        super().__init__()
        self.port = port
        self.piper = None


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
                    print("使能成功!!!!")
                else:
                    raise Exception("Failed to enable Piper")
                
                self.piper.MotionCtrl_1(0x00, 0x02, 0x01)
                # self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00) # CAN mode, Joint control, speed percent, mit mode (position, speed)
                # self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
                # self.piper.GripperCtrl(0, 1000,0x01, 0) 
                # #self.piper.GripperCtrl(round(0.08 * 1e6), 1000, 0x01, 0)
                
            else:
                raise Exception("Failed to connect to Piper")
            yield self.piper
        
        except Exception as e:
            print(f"Error: {e}")
        
        finally:
            if self.piper:
                
                # self.piper.MotionCtrl_1(0x00, 0x00, 0x02)
                
                flag = enable_fun(piper=self.piper, enable=False)
                if(flag == True):
                    print("失能成功!!!!")
                    exit(0)
        
                self.piper.DisconnectPort()
                self.piper = None