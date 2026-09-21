"""Launch only the calibrated, non-motion LAKSA planning preview stack."""

import atexit
import json
import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _positive_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)) and float(value) > 0.0


def _configured_nodes(context):
    geometry_path = Path(LaunchConfiguration("geometry_config").perform(context))
    template_path = Path(LaunchConfiguration("planner_config").perform(context))
    geometry = (yaml.safe_load(geometry_path.read_text(encoding="utf-8")) or {}).get("planning_geometry", {})
    required = ("wheelbase_m", "front_m", "rear_m", "left_m", "right_m")
    if geometry.get("calibrated") is not True or not all(_positive_number(geometry.get(name)) for name in required):
        raise RuntimeError(f"LAKSA planning geometry is incomplete or not approved: {geometry_path}")

    direct_radius = geometry.get("minimum_turning_radius_m")
    steering_deg = geometry.get("effective_max_road_wheel_steering_deg")
    if _positive_number(direct_radius):
        minimum_radius = float(direct_radius)
    elif _positive_number(steering_deg) and float(steering_deg) < 90.0:
        minimum_radius = float(geometry["wheelbase_m"]) / math.tan(math.radians(float(steering_deg)))
    else:
        raise RuntimeError("Set a measured minimum turning radius or a valid effective road-wheel steering angle")

    front = float(geometry["front_m"]); rear = float(geometry["rear_m"])
    left = float(geometry["left_m"]); right = float(geometry["right_m"])
    footprint = json.dumps([[front, left], [front, -right], [-rear, -right], [-rear, left]], separators=(",", ":"))
    parameters = template_path.read_text(encoding="utf-8")
    parameters = parameters.replace('"__LAKSA_MINIMUM_TURNING_RADIUS__"', f"{minimum_radius:.9f}")
    parameters = parameters.replace('"__LAKSA_FOOTPRINT__"', json.dumps(footprint))
    if "__LAKSA_" in parameters:
        raise RuntimeError("Unresolved LAKSA planning parameter in planner template")

    handle = tempfile.NamedTemporaryFile(mode="w", prefix="laksa_planning_preview_", suffix=".yaml", delete=False)
    handle.write(parameters); handle.close()
    generated_path = Path(handle.name)
    atexit.register(lambda: generated_path.unlink(missing_ok=True))

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[str(generated_path)],
    )
    accepted_controller_config = (
        Path(get_package_share_directory("laksa_bringup")) / "config" / "nav2_ackermann.yaml"
    )
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[str(accepted_controller_config)],
        remappings=[("cmd_vel", "/laksa/nav_cmd_vel")],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="laksa_planning_preview_lifecycle_manager",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["planner_server", "controller_server"]}],
    )
    return [planner, controller, lifecycle]


def generate_launch_description():
    share = Path(get_package_share_directory("laksa_dashboard"))
    return LaunchDescription([
        DeclareLaunchArgument("geometry_config", default_value=str(share / "config" / "planning_geometry.yaml")),
        DeclareLaunchArgument("planner_config", default_value=str(share / "config" / "planning_preview_nav2.yaml")),
        OpaqueFunction(function=_configured_nodes),
    ])
