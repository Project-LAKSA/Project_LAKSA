"""Canonical saved-map navigation using only official Nav2 core nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dashboard_share = Path(get_package_share_directory("laksa_dashboard"))
    bringup_share = Path(get_package_share_directory("laksa_bringup"))
    mapping_share = Path(get_package_share_directory("laksa_mapping"))
    zed_share = Path(get_package_share_directory("zed_wrapper"))
    map_yaml = LaunchConfiguration("map_yaml")
    zed_params_override_path = LaunchConfiguration("zed_params_override_path")

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(zed_share / "launch" / "zed_camera.launch.py")),
        launch_arguments={
            "camera_model": "zed2i",
            "camera_name": "zed",
            "publish_urdf": "true",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "publish_imu_tf": "true",
            "ros_params_override_path": zed_params_override_path,
            "node_log_type": "screen",
        }.items(),
    )
    zed_base_pose = Node(
        package="laksa_mapping", executable="zed_base_pose_adapter",
        name="zed_base_pose_adapter", output="screen",
        respawn=True, respawn_delay=2.0,
    )
    measurements = Node(
        package="laksa_bringup", executable="state_measurements_node.py",
        name="laksa_navigation_state_measurements", output="screen",
        parameters=[{"odom_frame": "odom", "base_frame": "base_footprint"}],
        respawn=True, respawn_delay=2.0,
    )
    ekf = Node(
        package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[str(mapping_share / "config" / "flat_ground_fusion.yaml")],
        remappings=[("odometry/filtered", "/laksa/odometry/fused")],
    )
    localization_config = str(dashboard_share / "config" / "saved_map_localization.yaml")
    navigation_config = str(bringup_share / "config" / "nav2_ackermann.yaml")
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[localization_config, {"yaml_filename": map_yaml}],
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[localization_config],
        remappings=[("scan", "/laksa/lidar/scan_validated")],
    )
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        output="screen", parameters=[navigation_config],
    )
    controller = Node(
        package="nav2_controller", executable="controller_server", name="controller_server",
        output="screen", parameters=[navigation_config],
        remappings=[("cmd_vel", "/laksa/nav_cmd_vel")],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="laksa_saved_navigation_lifecycle_manager", output="screen",
        parameters=[{
            "autostart": True,
            "bond_timeout": 4.0,
            "node_names": ["map_server", "amcl", "planner_server", "controller_server"],
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument(
            "zed_params_override_path",
            default_value=str(dashboard_share / "config" / "navigation_zed_timing.yaml"),
            description="Navigation-only ZED timing profile; optional diagnostic override",
        ),
        zed,
        measurements,
        zed_base_pose,
        ekf,
        map_server,
        amcl,
        planner,
        controller,
        lifecycle,
    ])
