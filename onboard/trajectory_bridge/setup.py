from setuptools import setup

package_name = "g1_d_trajectory_bridge"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="unitree@localhost",
    description="Split-process Foxy trajectory bridge for Unitree G1-D",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "trajectory_backend = g1_d_trajectory_bridge.trajectory_backend:main",
            "trajectory_frontend = g1_d_trajectory_bridge.trajectory_frontend:main",
            "trajectory_executor = g1_d_trajectory_bridge.trajectory_executor_blocked:main",
            "trajectory_topic_dry_run = g1_d_trajectory_bridge.trajectory_topic_dry_run:main",
        ],
    },
)
