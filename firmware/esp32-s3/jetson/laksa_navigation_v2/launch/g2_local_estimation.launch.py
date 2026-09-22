"""Production-intent, observational local-odometry graph.

It deliberately starts neither a sensor driver nor any navigation, mapping,
controller, or actuator component. A hardware bringup supplies raw ZED VIO,
VESC telemetry, and robot_state_publisher mechanical TF separately.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    speed_enabled = LaunchConfiguration("enable_vehicle_speed_fusion")
    common = {
        "package": "robot_localization",
        "executable": "ekf_node",
        "name": "ekf_local_odom",
        "remappings": [("odometry/filtered", "/laksa/odometry/local")],
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_vehicle_speed_fusion", default_value="false",
            description="Requires a measured physical VESC speed covariance; false is the safe default."),
        DeclareLaunchArgument("vesc_speed_variance_mps2", default_value="-1.0"),
        Node(**common, condition=UnlessCondition(speed_enabled), parameters=[str(package_root / "config" / "ekf_local_odom.yaml")]),
        Node(**common, condition=IfCondition(speed_enabled), parameters=[str(package_root / "config" / "ekf_local_odom_speed_enabled.yaml")]),
        Node(package="laksa_navigation_v2", executable="vio_base_odometry_adapter_node", name="vio_base_odometry_adapter"),
        Node(
            package="laksa_navigation_v2", executable="vehicle_speed_adapter_node",
            name="vehicle_speed_adapter",
            parameters=[{"variance_mps2": LaunchConfiguration("vesc_speed_variance_mps2")}],
        ),
        Node(
            package="laksa_navigation_v2", executable="local_odometry_contract_monitor",
            name="local_odometry_contract_monitor",
        ),
    ])
