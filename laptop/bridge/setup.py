from setuptools import setup

package_name = "g1_d_moveit_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yii",
    maintainer_email="yii@localhost",
    description="Laptop-side MoveIt trajectory relay for Unitree G1-D",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "trajectory_action_relay = g1_d_moveit_bridge.trajectory_action_relay:main",
            "trajectory_action_relay_dry_run = g1_d_moveit_bridge.trajectory_action_relay_dry_run:main",
        ],
    },
)
