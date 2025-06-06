import time
from contextlib import contextmanager
from threading import Lock

from pynput.keyboard import KeyCode, Listener


class Keyboard(Listener):
    def __init__(self):
        self.key_press_list = list()
        self.lock = Lock()
        super().__init__(on_press=self.on_press, on_release=self.on_release)

    def on_press(self, key):
        with self.lock:
            self.key_press_list.append(key)

    def on_release(self, key):
        pass

    def clear(self):
        with self.lock:
            self.key_press_list = list()

    def get_press_events(self):
        with self.lock:
            events = list(self.key_press_list)
            self.key_press_list = list()
            return events

    @contextmanager
    def activate(self):
        try:
            with self:
                yield self
        finally:
            pass


def wait_keyboard_response(keys, keyboard, dt=1e-3):
    if isinstance(keys, str):
        keys = [keys]
    while True:
        press_events = keyboard.get_press_events()
        for key_stroke in press_events:
            for key in keys:
                if key_stroke == KeyCode(char=key):
                    return key
        time.sleep(dt)
