from piper.teleop.device.mouse import MouseReader
import time 

# Example usage:
if __name__ == "__main__":
    with MouseReader() as mouse_reader:        
        try:
            while True:
                print(mouse_reader.read())
                time.sleep(0.1)
        except KeyboardInterrupt:
            mouse_reader.stop()
            print("MouseReader stopped.")


# from pynput.mouse import Listener
# def on_click(x, y, button, pressed):
#     if pressed:
#        print(button)
   
# # Collect events until released
# with Listener(on_click=on_click) as listener:
#     listener.join()