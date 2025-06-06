import time
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop
from lerobot.common.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


if __name__ == "__main__":
    keyboard_teleop = KeyboardTeleop(KeyboardTeleopConfig())
    keyboard_teleop.connect()
    while True:
        action = keyboard_teleop.get_action()
        print(action)
        time.sleep(0.1)