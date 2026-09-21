"""Start the Xbox-first supervisor with autonomy capability disarmed."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("laksa_bringup")) / "config" / "drive_supervisor.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "device_id",
            default_value="0",
            description="SDL Xbox controller device index",
        ),
        Node(
            package="joy",
            executable="game_controller_node",
            name="joy_node",
            output="screen",
            parameters=[str(config), {"device_id": LaunchConfiguration("device_id")}],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="laksa_bringup",
            executable="drive_supervisor_node.py",
            name="drive_supervisor",
            output="screen",
            parameters=[
                str(config),
                # Capability is available, but DriveSupervisor always starts
                # DISARMED. Only its explicit SetBool service may arm it.
                {"autonomy_enabled": True, "actuation_enabled": True},
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
