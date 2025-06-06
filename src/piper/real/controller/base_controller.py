from contextlib import contextmanager
from abc import ABC

class BaseController(ABC):
    def schedule_waypoint(self, target_pose, target_time):
        raise NotImplementedError("Subclasses must implement this method")
    
    def get_state(self):
        raise NotImplementedError("Subclasses must implement this method")
    
    def get_all_state(self):
        raise NotImplementedError("Subclasses must implement this method")
    
    @contextmanager
    def activate(self):
        try:
            yield self
        finally:
            pass