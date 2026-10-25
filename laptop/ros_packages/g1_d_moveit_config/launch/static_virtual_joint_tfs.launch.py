from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 升降柱补偿高度
    # 当前机器人已经升高约15cm
    lift_height = 0.127

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_virtual_joint_tf_lifted",
        arguments=[
            "0.0",
            "0.0",
            str(lift_height),
            "0.0",
            "0.0",
            "0.0",
            "world",
            "AGV_link",
        ],
    )

    return LaunchDescription([
        static_tf
    ])
