from contextlib import contextmanager
from piper_sdk import C_PiperInterface_V2  

class Piper:
    def __init__(self, port: str = "can0", no_real: bool = False):
        self.piper = None
        self.port = port
        self.no_real = no_real # if True, the Piper will not be connected to the real robot

    def set_no_real(self):
        self.no_real = True

    @contextmanager
    def activate(self):
        """
        Activate the Piper SDK.
        """
        try:
            self.piper = C_PiperInterface_V2(self.port)
            self.piper.ConnectPort(True)
            if self.piper:
                version = self.piper.GetPiperFirmwareVersion()
                print(f"Connected to Piper with SDK version: {version}")
            else:
                raise Exception("Failed to connect to Piper")
            yield self.piper
        finally:
            if self.piper:
                self.piper.DisconnectPort()
                self.piper = None

    def observe(self):
        """
        Observe the Piper.
        """
        pass

    def act(self):
        """
        Act on the Piper.
        """
        pass
            