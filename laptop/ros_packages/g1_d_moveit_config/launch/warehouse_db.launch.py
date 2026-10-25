from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("G1_D", package_name="g1_d_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
