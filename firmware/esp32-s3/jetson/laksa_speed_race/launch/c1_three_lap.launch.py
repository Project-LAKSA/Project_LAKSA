"""Launch the simulation-only C1 Gym adapter and pinned Waterloo controller."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("laksa_speed_race"))
    controller = yaml.safe_load((share / "config" / "c1_pure_pursuit.yaml").read_text())
    controller_params = controller["pure_pursuit"]["ros__parameters"]
    controller_params["waypoints_path"] = str(
        share / "course" / "canonical" / "speed_course" / "pure_pursuit_raceline.csv"
    )
    output_dir = LaunchConfiguration("output_dir")
    adapter = Node(
        package="laksa_speed_race",
        executable="c1_gym_adapter",
        name="c1_gym_adapter",
        output="screen",
        parameters=[
            {
                "output_dir": output_dir,
                "max_laps": ParameterValue(LaunchConfiguration("max_laps"), value_type=int),
            }
        ],
    )
    pure_pursuit = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[controller_params],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("max_laps", default_value="3"),
            DeclareLaunchArgument("output_dir", default_value="/tmp/laksa-c1-results/official"),
            adapter,
            pure_pursuit,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=adapter,
                    on_exit=[EmitEvent(event=Shutdown(reason="C1 adapter reached terminal state"))],
                )
            ),
        ]
    )
