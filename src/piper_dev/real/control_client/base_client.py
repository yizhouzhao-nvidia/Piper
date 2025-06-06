from contextlib import contextmanager
from abc import ABC

class BaseClient(ABC):
    def __init__(self):
        self.active = False
    
    def move(self, cmds):
        raise NotImplementedError("Subclasses must implement this method")
    
    def get(self):
        raise NotImplementedError("Subclasses must implement this method")
    
    @contextmanager
    def activate(self):
        raise NotImplementedError("Subclasses must implement this method")
    
    @property
    def cmd_shape(self):
        raise NotImplementedError("Subclasses must implement this property")
    
    @property
    def state_shape(self):
        raise NotImplementedError("Subclasses must implement this property")
        