from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("laksa_lidar"))
    use_existing_scan = LaunchConfiguration("use_existing_scan")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")

    driver = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        condition=UnlessCondition(use_existing_scan),
        parameters=[{
            "channel_type": "serial",
            "serial_port": LaunchConfiguration("serial_port"),
            "serial_baudrate": 256000,
            "frame_id": LaunchConfiguration("lidar_frame"),
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Sensitivity",
            "scan_frequency": 10.0,
        }],
        remappings=[("scan", raw_scan_topic)],
    )
    guard = Node(
        package="laksa_lidar",
        executable="lidar_guard",
        name="lidar_guard",
        output="screen",
        parameters=[str(share / "config" / "a2m12.yaml"), {
            "raw_scan_topic": raw_scan_topic,
            "target_frame": LaunchConfiguration("target_frame"),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_existing_scan", default_value="true",
            description="Use an already-owned raw scan publisher; false starts exactly one sllidar driver.",
        ),
        DeclareLaunchArgument("raw_scan_topic", default_value="/scan_raw"),
        DeclareLaunchArgument("target_frame", default_value="base_footprint"),
        DeclareLaunchArgument("serial_port", default_value="/dev/laksa_lidar"),
        DeclareLaunchArgument("lidar_frame", default_value="lidar_link"),
        driver,
        guard,
    ])
