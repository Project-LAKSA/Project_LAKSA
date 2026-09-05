from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="laksa_mapping", executable="mapping_session_manager", name="mapping_session_manager", output="screen"),
        Node(package="laksa_health", executable="health_monitor", name="laksa_health_monitor", output="screen"),
        Node(package="laksa_dashboard", executable="cockpit_server", name="laksa_mapping_cockpit", parameters=[{"port": 8090, "max_cloud_points": 10000, "cloud_period_sec": 1.0}], output="screen"),
        Node(package="web_video_server", executable="web_video_server", name="laksa_mapping_video", parameters=[{"port": 8080}], output="screen"),
    ])
