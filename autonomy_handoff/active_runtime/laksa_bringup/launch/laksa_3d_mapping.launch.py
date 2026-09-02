"""Launch calibrated ZED RGB-D, RPLIDAR, fused odometry, and RTAB-Map."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _finite_number(mount, key: str) -> float:
    value = float(mount[key])
    if not math.isfinite(value):
        raise RuntimeError(f"camera mount value {key} is not finite")
    return value


def _build_calibrated_stack(context):
    mount_path = Path(LaunchConfiguration("camera_mount_file").perform(context))
    try:
        document = yaml.safe_load(mount_path.read_text(encoding="utf-8"))
        mount = document["camera_mount"]
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise RuntimeError(f"invalid ZED mount file {mount_path}: {error}") from error
    if mount.get("configured") is not True:
        raise RuntimeError(
            "ZED 3-D mapping is calibration-locked: set camera_mount.configured "
            "to true only after measuring x/y/z/roll/pitch/yaw"
        )

    parent = str(mount["parent_frame"]).strip()
    child = str(mount["child_frame"]).strip()
    if parent != "laksa_base_footprint" or child != "zed_camera_link":
        raise RuntimeError(
            "camera mount frames must be laksa_base_footprint -> zed_camera_link"
        )
    x = _finite_number(mount, "x_m")
    y = _finite_number(mount, "y_m")
    z = _finite_number(mount, "z_m")
    roll = _finite_number(mount, "roll_rad")
    pitch = _finite_number(mount, "pitch_rad")
    yaw = _finite_number(mount, "yaw_rad")
    if max(abs(x), abs(y), abs(z)) > 2.0:
        raise RuntimeError("camera translation exceeds the 2 m sanity bound")
    if max(abs(roll), abs(pitch), abs(yaw)) > math.tau:
        raise RuntimeError("camera rotation must be expressed in radians")

    share = Path(get_package_share_directory("laksa_bringup"))
    zed_share = Path(get_package_share_directory("zed_wrapper"))
    rtabmap_share = Path(get_package_share_directory("rtabmap_launch"))
    serial_port = LaunchConfiguration("serial_port")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "channel_type": "serial",
            "serial_port": serial_port,
            "serial_baudrate": 256000,
            "frame_id": "laksa_lidar",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Sensitivity",
            "scan_frequency": 10.0,
        }],
        remappings=[("scan", raw_scan_topic)],
    )
    lidar_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laksa_lidar_transform",
        output="screen",
        arguments=[
            "--x", "0.31542", "--y", "0.0", "--z", "0.13542",
            "--yaw", "3.14159265", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "laksa_base_footprint",
            "--child-frame-id", "laksa_lidar",
        ],
    )
    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        output="screen",
        parameters=[str(share / "config" / "lidar_filters.yaml")],
        remappings=[("scan", raw_scan_topic), ("scan_filtered", scan_topic)],
    )
    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "laser_scan_topic": scan_topic,
            "odom_topic": "/laksa/rf2o_odom",
            "base_frame_id": "laksa_base_footprint",
            "odom_frame_id": "laksa_odom",
            "publish_tf": False,
            "init_pose_from_topic": "",
            "freq": 10.0,
        }],
    )
    watchdog = Node(
        package="laksa_bringup",
        executable="lidar_watchdog_node.py",
        name="lidar_watchdog",
        output="screen",
        parameters=[{
            "scan_topic": raw_scan_topic,
            "startup_grace_sec": 2.0,
            "scan_timeout_sec": 2.0,
            "retry_period_sec": 2.0,
            "max_start_attempts": 3,
        }],
        on_exit=[Shutdown(reason="LiDAR watchdog requested a stack restart")],
    )

    zed_mount_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laksa_zed_mount_transform",
        output="screen",
        arguments=[
            "--x", str(x), "--y", str(y), "--z", str(z),
            "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
            "--frame-id", parent, "--child-frame-id", child,
        ],
    )
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(zed_share / "launch" / "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_name": "zed",
            "camera_model": "zed2i",
            "publish_urdf": "true",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "publish_imu_tf": "false",
            "ros_params_override_path": str(
                share / "config" / "zed2i_robot.yaml"
            ),
            "node_log_type": "screen",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_zed")),
    )
    measurements = Node(
        package="laksa_bringup",
        executable="state_measurements_node.py",
        name="state_measurements",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[str(share / "config" / "sensor_fusion.yaml")],
        remappings=[("odometry/filtered", "/laksa/odom")],
    )

    rtabmap_args = " ".join([
        "--Reg/Strategy 2",
        "--RGBD/NeighborLinkRefining true",
        "--RGBD/ProximityBySpace true",
        "--RGBD/LinearUpdate 0.08",
        "--RGBD/AngularUpdate 0.05",
        # Build the persistent occupancy representation from both the planar
        # LiDAR and ZED depth. Grid/3D is required for the OctoMap outputs;
        # Nav2 still consumes the projected 2-D /map independently.
        "--Grid/Sensor 2",
        "--Grid/3D true",
        "--Grid/RayTracing true",
        "--Grid/CellSize 0.05",
        "--Grid/RangeMax 8.0",
        "--Grid/FootprintLength 0.57",
        "--Grid/FootprintWidth 0.36",
        "--Grid/FootprintHeight 0.30",
        "--Grid/MaxGroundHeight 0.08",
        "--Grid/MaxObstacleHeight 1.50",
        "--RGBD/ProximityPathMaxNeighbors 10",
        "--Mem/DepthCompressionFormat .png",
        "--Rtabmap/DetectionRate 1.0",
        # map_always_update is a ROS parameter of rtabmap_ros (not an
        # RTAB-Map core argument and not a declared include-launch argument).
        "--ros-args -p map_always_update:=true",
    ])
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rtabmap_share / "launch" / "rtabmap.launch.py")
        ),
        launch_arguments={
            "stereo": "false",
            "localization": LaunchConfiguration("localization"),
            "rtabmap_viz": "false",
            "rviz": "false",
            "frame_id": "laksa_base_footprint",
            "map_frame_id": "map",
            "map_topic": "/map",
            "publish_tf_map": "true",
            "rgb_topic": "/zed/zed_node/rgb/color/rect/image",
            "depth_topic": "/zed/zed_node/depth/depth_registered",
            "camera_info_topic":
                "/zed/zed_node/rgb/color/rect/image/camera_info",
            "approx_sync": "true",
            "approx_sync_max_interval": "0.15",
            # Bound latency under load: dropping an old frame is preferable to
            # navigating with a transform/image pair more than a second stale.
            "topic_queue_size": "5",
            "sync_queue_size": "10",
            "wait_for_transform": "1.2",
            "subscribe_scan": "true",
            "scan_topic": scan_topic,
            "visual_odometry": "false",
            "icp_odometry": "false",
            "odom_topic": "/laksa/odom",
            "publish_tf_odom": "false",
            "qos": "2",
            "database_path": LaunchConfiguration("database_path"),
            "args": rtabmap_args,
        }.items(),
    )
    return [
        lidar,
        lidar_transform,
        scan_filter,
        rf2o,
        watchdog,
        zed_mount_transform,
        zed,
        measurements,
        ekf,
        rtabmap,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("laksa_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/laksa_lidar"),
        DeclareLaunchArgument("raw_scan_topic", default_value="/scan_raw"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "camera_mount_file",
            default_value=str(share / "config" / "zed_mount.yaml"),
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value="/home/ubuntu/.ros/laksa_rtabmap.db",
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description="Use an existing RTAB-Map database without extending it",
        ),
        DeclareLaunchArgument(
            "start_zed",
            default_value="false",
            description=(
                "Start the ZED wrapper inside this launch. The boot service "
                "normally provides the camera, so the default is false."
            ),
        ),
        OpaqueFunction(function=_build_calibrated_stack),
    ])
