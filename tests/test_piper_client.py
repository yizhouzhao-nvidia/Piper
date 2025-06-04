import time

from piper_dev.control_client.piper_client import PiperClient

if __name__ == "__main__":
    piper_client = PiperClient()
    begin_time = time.time()
    with piper_client.activate():
        print("piper_client activated")
        while time.time() - begin_time <= 5:
            print("joint_state:", piper_client.get())
            time.sleep(0.1)
        
    print("piper_client deactivated")
    
        
