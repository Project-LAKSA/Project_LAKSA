"""Isolated experimental ZED RGB-D + validated A2M12 RTAB-Map session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    zed_share = get_package_share_directory("zed_wrapper")
    mapping_share = get_package_share_directory("laksa_mapping")
    zed_config = LaunchConfiguration("zed_config")
    rtab_config = LaunchConfiguration("rtab_config")

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(zed_share, "launch", "zed_camera.launch.py")),
        launch_arguments={
            "camera_model": "zed2i", "camera_name": "zed", "publish_urdf": "true",
            "publish_tf": "false", "publish_map_tf": "false", "publish_imu_tf": "true",
            "ros_params_override_path": zed_config, "node_log_type": "screen",
        }.items(),
    )
    measurements = Node(
        package="laksa_bringup", executable="state_measurements_node.py",
        name="laksa_mapping_state_measurements", output="screen",
        parameters=[{"odom_frame": "odom", "base_frame": "base_footprint"}],
        respawn=True, respawn_delay=2.0,
    )
    zed_base_pose = Node(
        package="laksa_mapping", executable="zed_base_pose_adapter",
        name="zed_base_pose_adapter", output="screen", respawn=True, respawn_delay=2.0,
    )
    ekf = Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
        parameters=[os.path.join(mapping_share, "config", "flat_ground_fusion.yaml")],
        remappings=[("odometry/filtered", "/laksa/odometry/fused")],
    )
    sync = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync",
        namespace="laksa/mapping_shadow",
        parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.05, "queue_size": 10, "qos": 2}],
        remappings=[
            ("rgb/image", "/zed/zed_node/rgb/color/rect/image"),
            ("rgb/camera_info", "/zed/zed_node/rgb/color/rect/image/camera_info"),
            ("depth/image", "/zed/zed_node/depth/depth_registered"),
        ],
    )
    rtab = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap",
        namespace="laksa/mapping_shadow", parameters=[rtab_config],
        remappings=[
            ("rgbd_image", "/laksa/mapping_shadow/rgbd_image"),
            ("odom", "/laksa/odometry/fused"),
            ("scan", "/laksa/lidar/scan_validated"),
        ],
        arguments=["-d"], output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("zed_config"), DeclareLaunchArgument("rtab_config"),
        zed, measurements, zed_base_pose, ekf, TimerAction(period=6.0, actions=[sync, rtab]),
    ])
