from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory("laksa_description")) / "urdf" / "laksa_visualization.urdf"
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="laksa_visualization_state_publisher",
            parameters=[{"robot_description": urdf.read_text(encoding="utf-8")}],
            output="screen",
        )
    ])
