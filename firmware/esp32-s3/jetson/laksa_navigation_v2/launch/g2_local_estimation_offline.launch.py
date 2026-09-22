"""Strictly test-only G2 graph: synthetic standard messages plus robot_localization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    enabled = LaunchConfiguration("enable_g2_test_only")
    return LaunchDescription([
        DeclareLaunchArgument("enable_g2_test_only", default_value="false"),
        DeclareLaunchArgument("scenario", default_value="G2_S001_STATIONARY"),
        GroupAction(condition=IfCondition(enabled), actions=[
            Node(package="robot_localization", executable="ekf_node", name="ekf_local_odom", parameters=[str(package_root / "config" / "ekf_local_odom_synthetic.yaml")], remappings=[("odometry/filtered", "/laksa/odometry/local")]),
            Node(package="laksa_navigation_v2", executable="g2_synthetic_sensor_node", name="g2_test_only_synthetic_inputs", parameters=[{"scenario": LaunchConfiguration("scenario")}]),
        ]),
    ])
