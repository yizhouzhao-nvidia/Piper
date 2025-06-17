from typing import Callable, Dict, Tuple, Any
from contextlib import contextmanager

#########################################################################


class Embodiment():
    """
    The embodiment/robot/agent in the scene. It can act and observe
    """

    action_space: Any
    observation_space: Any

    def act(self, action: Dict):
        """
        Perform the action in the scene
        """
        pass

    def observe(self) -> Dict:
        """
        Snapshot of the agent's observation
        """
        pass

    def register_obs_callback(self, callback: Callable):
        """
        Register a callback function to be called when the agent obtains
        the latest observation of the world
        """
        pass

    @contextmanager
    def activate(self):
        yield self

