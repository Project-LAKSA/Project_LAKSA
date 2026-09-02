"""Start the LAKSA RPLIDAR A2M12 and online 2-D mapping."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("laksa_bringup"))

    serial_port = LaunchConfiguration("serial_port")
    lidar_frame = LaunchConfiguration("lidar_frame")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    slam_params = LaunchConfiguration("slam_params")

    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": 256000,
                "frame_id": lidar_frame,
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
                "scan_frequency": 10.0,
            }
        ],
        remappings=[("scan", raw_scan_topic)],
    )

    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        output="screen",
        parameters=[str(package_share / "config" / "lidar_filters.yaml")],
        remappings=[
            ("scan", raw_scan_topic),
            ("scan_filtered", scan_topic),
        ],
    )

    rf2o_odometry = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "laser_scan_topic": scan_topic,
                "odom_topic": "/laksa/odom",
                "base_frame_id": "laksa_base_footprint",
                "odom_frame_id": "laksa_odom",
                "publish_tf": True,
                # RF2O otherwise waits forever for its default ground-truth
                # topic. LAKSA deliberately starts odometry at the origin.
                "init_pose_from_topic": "",
                # The A2M12 currently delivers about 13.3 scans/s. Running
                # RF2O slightly faster ensures its transform for a scan exists
                # before the next scan displaces it from SLAM's TF queue.
                "freq": 15.0,
            }
        ],
    )

    watchdog = Node(
        package="laksa_bringup",
        executable="lidar_watchdog_node.py",
        name="lidar_watchdog",
        output="screen",
        parameters=[
            {
                # Watch the driver directly so a transient TF/filter delay does
                # not unnecessarily restart a healthy RPLIDAR.
                "scan_topic": raw_scan_topic,
                "startup_grace_sec": 2.0,
                "scan_timeout_sec": 2.0,
                "retry_period_sec": 2.0,
                "max_start_attempts": 3,
            }
        ],
        on_exit=[Shutdown(reason="LiDAR watchdog requested a full stack restart")],
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
    )

    lidar_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laksa_lidar_transform",
        output="screen",
        arguments=[
            "--x", "0.31542",
            "--y", "0.0",
            "--z", "0.13542",
            "--yaw", "3.14159265",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "laksa_base_footprint",
            "--child-frame-id", "laksa_lidar",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/laksa_lidar",
                description="Stable udev link for the RPLIDAR serial adapter",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="laksa_lidar",
                description="TF frame used in LaserScan messages",
            ),
            DeclareLaunchArgument(
                "raw_scan_topic",
                default_value="/scan_raw",
                description="Unfiltered LaserScan topic from the RPLIDAR",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="Self-filtered LaserScan consumed by SLAM and Nav2",
            ),
            DeclareLaunchArgument(
                "slam_params",
                default_value=str(package_share / "config" / "lidar_mapping.yaml"),
                description="SLAM Toolbox parameter file",
            ),
            lidar,
            lidar_transform,
            scan_filter,
            rf2o_odometry,
            watchdog,
            slam,
        ]
    )
