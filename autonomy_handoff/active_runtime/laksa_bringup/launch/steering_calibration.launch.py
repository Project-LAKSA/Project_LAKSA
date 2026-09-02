from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("laksa_bringup"))
        / "config"
        / "steering_calibration.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="SDL game-controller device index",
            ),
            Node(
                package="joy",
                executable="game_controller_node",
                name="joy_node",
                output="screen",
                parameters=[str(config), {"device_id": LaunchConfiguration("device_id")}],
            ),
            Node(
                package="laksa_bringup",
                executable="steering_calibration_node.py",
                name="steering_calibration_node",
                output="screen",
                parameters=[str(config)],
            ),
        ]
    )
