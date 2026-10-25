from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "G1_D",
            package_name="g1_d_moveit_config",
        )
        .to_moveit_configs()
    )

    launch_path = moveit_config.package_path / "launch"

    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_path / "static_virtual_joint_tfs.launch.py")
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_path / "rsp.launch.py")
        )
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_path / "move_group.launch.py")
        )
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_path / "moveit_rviz.launch.py")
        )
    )

    return LaunchDescription([
        static_virtual_joint_tfs,
        robot_state_publisher,
        move_group,
        # move_group needs a moment to advertise /get_planning_scene.
        # Starting RViz immediately can leave MotionPlanning stuck at
        # "Requesting initial scene failed" even though the service appears
        # shortly afterwards.
        TimerAction(period=3.0, actions=[moveit_rviz]),
    ])
