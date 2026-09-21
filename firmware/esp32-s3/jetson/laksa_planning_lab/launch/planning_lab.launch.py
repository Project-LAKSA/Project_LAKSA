"""Planner-only LAKSA lab stack. It contains no vehicle-motion components."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params_file")
    use_smoother = LaunchConfiguration("use_smoother")
    base_x = LaunchConfiguration("base_x")
    base_y = LaunchConfiguration("base_y")
    base_yaw = LaunchConfiguration("base_yaw")
    namespace = "laksa_planning_lab"
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("use_smoother", default_value="false"),
        # Planner-lab-only robot pose.  The official IsPathValid service starts
        # checking at the path point closest to this transform, so forensic
        # callers set it to the exact requested start pose.
        DeclareLaunchArgument("base_x", default_value="0"),
        DeclareLaunchArgument("base_y", default_value="0"),
        DeclareLaunchArgument("base_yaw", default_value="0"),
        Node(package="nav2_map_server", executable="map_server", name="map_server", namespace=namespace,
             output="screen", parameters=[params, {"yaml_filename": map_yaml, "topic_name": "map", "frame_id": "map"}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", namespace=namespace,
             output="screen", parameters=[params]),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", namespace=namespace,
             output="screen", parameters=[params], condition=IfCondition(use_smoother),
             remappings=[("global_costmap/costmap_raw", "/laksa_planning_lab/global_costmap/costmap_raw"),
                         ("global_costmap/published_footprint", "/laksa_planning_lab/global_costmap/published_footprint")]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="lab_static_tf", namespace=namespace,
             arguments=[base_x, base_y, "0", "0", "0", base_yaw, "map", "base_footprint"]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="lab_base_link_tf", namespace=namespace,
             arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager", namespace=namespace,
             output="screen", parameters=[{"use_sim_time": False, "autostart": True,
                                            "node_names": ["map_server", "planner_server", "smoother_server"]}],
             condition=IfCondition(use_smoother)),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager", namespace=namespace,
             output="screen", parameters=[{"use_sim_time": False, "autostart": True,
                                            "node_names": ["map_server", "planner_server"]}],
             condition=UnlessCondition(use_smoother)),
    ])
