from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory("laksa_description")) / "launch" / "description.launch.py"
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description))),
        Node(package="laksa_lidar", executable="lidar_guard", name="lidar_guard", output="screen"),
        Node(package="laksa_mapping", executable="mapping_session_manager", name="mapping_session_manager", output="screen"),
        Node(package="laksa_health", executable="health_monitor", name="laksa_health_monitor", output="screen"),
        Node(package="laksa_dashboard", executable="cockpit_server", name="laksa_mapping_cockpit", parameters=[{"port": 8090, "max_cloud_points": 15000, "cloud_period_sec": 1.0}], output="screen"),
        Node(package="web_video_server", executable="web_video_server", name="laksa_mapping_video", parameters=[{"port": 8080}], output="screen"),
    ])
