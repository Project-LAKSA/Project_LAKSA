import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    zed_share = get_package_share_directory("zed_wrapper")
    description_share = get_package_share_directory("laksa_description")
    zed_config = LaunchConfiguration("zed_config")
    rtab_config = LaunchConfiguration("rtab_config")

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(zed_share, "launch", "zed_camera.launch.py")),
        launch_arguments={
            "camera_model": "zed2i",
            "camera_name": "zed",
            "publish_urdf": "true",
            "publish_tf": "true",
            "publish_map_tf": "false",
            "publish_imu_tf": "true",
            "ros_params_override_path": zed_config,
            "node_log_type": "screen",
        }.items(),
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, "launch", "description.launch.py"))
    )
    sync = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync", namespace="zed_rtabmap",
        parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.05, "queue_size": 10, "qos": 2}],
        remappings=[
            ("rgb/image", "/zed/zed_node/rgb/color/rect/image"),
            ("rgb/camera_info", "/zed/zed_node/rgb/color/rect/image/camera_info"),
            ("depth/image", "/zed/zed_node/depth/depth_registered"),
        ],
    )
    rtab = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="zed_rtabmap",
        parameters=[rtab_config],
        remappings=[("rgbd_image", "/zed_rtabmap/rgbd_image"), ("odom", "/zed/zed_node/odom")],
        arguments=["-d"], output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("zed_config"),
        DeclareLaunchArgument("rtab_config"),
        description,
        zed,
        TimerAction(period=6.0, actions=[sync, rtab]),
    ])
