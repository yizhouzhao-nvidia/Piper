from piper_dev.can_helper import find_can, activate_can

def main():
    print("Hello from piper-dev!")
    ports = find_can()
    activate_can(ports[0])

if __name__ == "__main__":
    main()
