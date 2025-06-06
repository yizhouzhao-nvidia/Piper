import abc

from lerobotdemo.robots.robot import Robot

class PreProcessor(abc.ABC):
    def __init__(self, **kwargs):
        pass

    def register(self, robot: Robot):
        self.robot = robot

    @abc.abstractmethod
    def __call__(self, data) -> dict:
        pass
