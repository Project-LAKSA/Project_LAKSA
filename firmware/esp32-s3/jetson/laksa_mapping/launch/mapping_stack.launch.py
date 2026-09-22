import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    zed_share = get_package_share_directory("zed_wrapper")
    mapping_share = get_package_share_directory("laksa_mapping")
    zed_config = LaunchConfiguration("zed_config")
    rtab_config = LaunchConfiguration("rtab_config")
    compose_rgbd_rtab = LaunchConfiguration("compose_rgbd_rtab")
    enable_dense_cloud_map = LaunchConfiguration("enable_dense_cloud_map")

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(zed_share, "launch", "zed_camera.launch.py")),
        launch_arguments={
            "camera_model": "zed2i",
            "camera_name": "zed",
            "publish_urdf": "true",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "publish_imu_tf": "true",
            "ros_params_override_path": zed_config,
            "node_log_type": "screen",
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
        name="zed_base_pose_adapter", output="screen",
        respawn=True, respawn_delay=2.0,
    )
    ekf = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[os.path.join(mapping_share, "config", "flat_ground_fusion.yaml")],
        remappings=[("odometry/filtered", "/laksa/odometry/fused")],
    )
    sync_parameters = [{"approx_sync": True, "approx_sync_max_interval": 0.05, "queue_size": 10, "qos": 2}]
    sync_remappings = [
        ("rgb/image", "/zed/zed_node/rgb/color/rect/image"),
        ("rgb/camera_info", "/zed/zed_node/rgb/color/rect/camera_info"),
        ("depth/image", "/zed/zed_node/depth/depth_registered"),
    ]
    rtab_parameters = [rtab_config]
    rtab_remappings = [
        ("rgbd_image", "/laksa/fused_mapping/rgbd_image"),
        ("scan", "/laksa/lidar/scan_validated"),
        ("odom", "/laksa/odometry/fused"),
        ("map", "/map"),
    ]

    sync = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync", namespace="laksa/fused_mapping",
        parameters=sync_parameters, remappings=sync_remappings,
        condition=UnlessCondition(compose_rgbd_rtab),
    )
    rtab = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", namespace="laksa/fused_mapping",
        parameters=rtab_parameters, remappings=rtab_remappings,
        arguments=["-d"], output="screen", condition=UnlessCondition(compose_rgbd_rtab),
    )
    composed = ComposableNodeContainer(
        package="rclcpp_components", executable="component_container_mt",
        name="rgbd_rtab_container", namespace="laksa/fused_mapping", output="screen",
        condition=IfCondition(compose_rgbd_rtab),
        composable_node_descriptions=[
            ComposableNode(
                package="rtabmap_sync", plugin="rtabmap_sync::RGBDSync",
                name="rgbd_sync", namespace="laksa/fused_mapping",
                parameters=sync_parameters, remappings=sync_remappings,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="rtabmap_slam", plugin="rtabmap_slam::CoreWrapper",
                name="rtabmap", namespace="laksa/fused_mapping",
                parameters=rtab_parameters, remappings=rtab_remappings,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )
    dense_cloud_generator = Node(
        package="rtabmap_util", executable="point_cloud_xyzrgb",
        name="dense_cloud_generator", namespace="laksa/fused_mapping", output="screen",
        condition=IfCondition(enable_dense_cloud_map),
        parameters=[{
            "decimation": 4,
            "min_depth": 0.3,
            "max_depth": 8.0,
            "voxel_size": 0.02,
            "filter_nans": True,
            "qos": 2,
        }],
        remappings=[
            ("rgbd_image", "/laksa/fused_mapping/rgbd_image"),
            ("cloud", "/laksa/fused_mapping/dense_cloud_local"),
        ],
    )
    dense_cloud_assembler = Node(
        package="rtabmap_util", executable="point_cloud_assembler",
        name="dense_cloud_assembler", namespace="laksa/fused_mapping", output="screen",
        condition=IfCondition(enable_dense_cloud_map),
        parameters=[{
            "fixed_frame_id": "map",
            "frame_id": "map",
            "max_clouds": 50,
            "circular_buffer": True,
            "linear_update": 0.10,
            "angular_update": 0.08,
            "range_min": 0.3,
            "range_max": 8.0,
            "voxel_size": 0.02,
            "qos": 2,
            "topic_queue_size": 1,
        }],
        remappings=[
            ("cloud", "/laksa/fused_mapping/dense_cloud_local"),
            ("assembled_cloud", "/laksa/fused_mapping/dense_cloud_map"),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("zed_config"),
        DeclareLaunchArgument("rtab_config"),
        DeclareLaunchArgument("compose_rgbd_rtab", default_value="false"),
        DeclareLaunchArgument("enable_dense_cloud_map", default_value="false"),
        zed,
        measurements,
        zed_base_pose,
        ekf,
        TimerAction(period=6.0, actions=[
            sync, rtab, composed, dense_cloud_generator, dense_cloud_assembler,
        ]),
    ])
