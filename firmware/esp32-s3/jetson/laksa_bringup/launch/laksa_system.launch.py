"""Manual-first LAKSA control, forward LiDAR cruise, and Nav2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("laksa_bringup"))
    drive_config = str(share / "config" / "drive_supervisor.yaml")
    nav_config = str(share / "config" / "nav2_ackermann.yaml")
    bt_xml = str(share / "config" / "navigate_ackermann.xml")
    bt_through_xml = str(
        share / "config" / "navigate_through_poses_ackermann.xml"
    )
    web_root = str(share / "web")

    joy = Node(
        package="joy",
        executable="game_controller_node",
        name="joy_node",
        output="screen",
        parameters=[drive_config, {"device_id": LaunchConfiguration("device_id")}],
        respawn=True,
        respawn_delay=2.0,
    )
    supervisor = Node(
        package="laksa_bringup",
        executable="drive_supervisor_node.py",
        name="drive_supervisor",
        output="screen",
        parameters=[drive_config],
        respawn=True,
        respawn_delay=2.0,
    )
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav_config],
        remappings=[("cmd_vel", "/laksa/nav_cmd_vel")],
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav_config],
    )
    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav_config],
        # Recovery commands must pass through the same mode, brake-latch, and
        # command-timeout arbitration as normal controller output.
        remappings=[("cmd_vel", "/laksa/nav_cmd_vel")],
    )
    navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav_config,
            {
                "default_nav_to_pose_bt_xml": bt_xml,
                "default_nav_through_poses_bt_xml": bt_through_xml,
            },
        ],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[nav_config],
    )
    lidar_cruise = Node(
        package="laksa_bringup",
        executable="lidar_cruise_node.py",
        name="lidar_cruise",
        output="screen",
        parameters=[drive_config],
        respawn=True,
        respawn_delay=2.0,
    )
    dashboard = Node(
        package="laksa_bringup",
        executable="dashboard_node.py",
        name="laksa_dashboard",
        output="screen",
        parameters=[{"port": 8088, "web_root": web_root}],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="SDL Xbox controller device index",
            ),
            joy,
            supervisor,
            controller,
            planner,
            behavior,
            navigator,
            lifecycle,
            dashboard,
            lidar_cruise,
        ]
    )
