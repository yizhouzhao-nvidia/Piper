# Third Party
import torch
# cuRobo
from curobo.types.base import TensorDeviceType
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig

robot_file = "franka.yml"

# create a world from a dictionary of objects
# cuboid: {} # dictionary of objects that are cuboids
# mesh: {} # dictionary of objects that are meshes
world_config = {
    "cuboid": {
        "table": {"dims": [2, 2, 0.2], "pose": [0.4, 0.0, -0.1, 1, 0, 0, 0]},
        "cube_1": {"dims": [0.1, 0.1, 0.2], "pose": [0.4, 0.0, 0.5, 1, 0, 0, 0]},
    },
    "mesh": {
        "scene": {
            "pose": [1.5, 0.080, 1.6, 0.043, -0.471, 0.284, 0.834],
            "file_path": "scene/nvblox/srl_ur10_bins.obj",
        }
    },
}
tensor_args = TensorDeviceType()
config = RobotWorldConfig.load_from_config(world_model=world_config)
curobo_fn = RobotWorld(config)