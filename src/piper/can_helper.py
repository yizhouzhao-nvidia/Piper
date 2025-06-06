import time
import subprocess
import os
import re

def find_can():
    print("Finding CAN devices...")
    time.sleep(0.1)
    port_matches = []

    # Get the path to the script
    script_path = os.path.join(os.path.dirname(__file__), "scripts", "find_all_can_port.sh")

    # Run the script with sudo
    result = subprocess.run(['sudo', 'bash', script_path], capture_output=True, text=True)

    # Print output
    print("STDOUT:", result.stdout)
    
    # if result.stderr is not empty, print it
    if result.stderr:
        print("STDERR:", result.stderr)


    matches = re.findall(r'接口名称:\s*(\w+)\s*端口号:\s*([\w.-]+:\d+\.\d+)\s*是否已激活:\s*(\S+)', result.stdout)
    for match in matches:
        port_matches.append(match[1])

    # print the port_matches
    print("Found CAN devices:")
    for port in port_matches:
        print(port)

    return port_matches

def activate_can(port, name="can0", bitrate=1000000):
    print(f"Activating CAN device: {port}")

    # Get the path to the script
    script_path = os.path.join(os.path.dirname(__file__), "scripts", "can_activate.sh")

    # Run the script with sudo
    # e.g bash can_activate.sh can_piper 1000000 "1-2:1.0"
    result = subprocess.run(['sudo', 'bash', script_path, name, str(bitrate), port], capture_output=True, text=True)
    # Print output
    print("STDOUT:", result.stdout)
    
    # if result.stderr is not empty, print it
    if result.stderr:
        print("STDERR:", result.stderr)
        return False
    else:
        print("Successfully activated CAN device")
        return True

if __name__ == "__main__":
    find_can()