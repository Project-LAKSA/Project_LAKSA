"""Launch deterministic C1.2 with pinned Nav2 RPP and no physical interfaces."""

from pathlib import Path

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
    output_dir = LaunchConfiguration("output_dir")
    raceline = Node(
        package="laksa_speed_race",
        executable="c1_nav2_raceline",
        name="nav2_raceline",
        namespace="c1",
        output="screen",
    )
    host = Node(
        package="laksa_speed_race_nav2",
        executable="rpp_lockstep_host",
        name="rpp_lockstep_host",
        namespace="c1",
        output="screen",
        parameters=[
            str(share / "config" / "c1_nav2_rpp.yaml"),
            {
                "map_yaml_path": str(
                    share / "course" / "canonical" / "speed_course" / "speed_course_nav2.yaml"
                )
            },
        ],
    )
    ackermann = Node(
        package="laksa_speed_race",
        executable="c1_nav2_ackermann_adapter",
        name="nav2_ackermann_adapter",
        namespace="c1",
        output="screen",
        parameters=[{"output_dir": output_dir}],
    )
    gym = Node(
        package="laksa_speed_race",
        executable="c1_gym_adapter",
        name="c1_gym_adapter",
        output="screen",
        parameters=[
            {
                "output_dir": output_dir,
                "max_laps": ParameterValue(LaunchConfiguration("max_laps"), value_type=int),
                "require_state_stamp": True,
                "controller_repo": "ros-navigation/navigation2",
                "controller_sha": "a097086719c88f781aa59788eca29ac6ca5e56db",
                "controller_config": str(share / "config" / "c1_nav2_rpp.yaml"),
                "qualification_step_limit": ParameterValue(
                    LaunchConfiguration("qualification_step_limit"), value_type=int
                ),
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("max_laps", default_value="3"),
            DeclareLaunchArgument("output_dir", default_value="/tmp/laksa-c1-results/nav2_trial_1"),
            DeclareLaunchArgument("qualification_step_limit", default_value="-1"),
            raceline,
            host,
            ackermann,
            gym,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gym,
                    on_exit=[EmitEvent(event=Shutdown(reason="C1.2 Gym authority reached terminal state"))],
                )
            ),
        ]
    )
