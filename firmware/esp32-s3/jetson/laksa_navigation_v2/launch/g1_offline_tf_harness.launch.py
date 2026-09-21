"""G1 test-only synthetic TF launch; it is disabled unless explicitly enabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration("enable_g1_test_only_publishers")
    return LaunchDescription([
        DeclareLaunchArgument("enable_g1_test_only_publishers", default_value="false"),
        GroupAction(condition=IfCondition(enabled), actions=[
            Node(package="tf2_ros", executable="static_transform_publisher", name="g1_test_only_global_localization", arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
            Node(package="tf2_ros", executable="static_transform_publisher", name="g1_test_only_local_state_estimator", arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]),
            Node(package="tf2_ros", executable="static_transform_publisher", name="g1_test_only_body_attitude_adapter", arguments=["0", "0", "0.13", "0", "0", "0", "base_footprint", "base_link"]),
        ]),
    ])
