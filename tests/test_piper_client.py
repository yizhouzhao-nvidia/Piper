import time

from piper_dev.control_client.piper_client import PiperClient

if __name__ == "__main__":
    piper_client = PiperClient()
    with piper_client.activate():
        print("piper_client activated")
        time.sleep(5)
        
    print("piper_client deactivated")
    
        
