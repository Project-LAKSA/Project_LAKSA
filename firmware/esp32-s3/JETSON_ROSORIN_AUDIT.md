# ROSOrin Jetson audit and LAKSA integration design

Read-only inspection over SSH. No Jetson files or packages were modified.

## Current status after installing the corrected image

The latest read-only audit supersedes the two historical corruption findings
below. The corrected image is operational:

- no EXT4, checksum, I/O, NVMe, segfault or illegal-instruction matches in the
  current boot kernel log;
- filesystem state is `clean`;
- `dpkg --audit`, APT configuration and `apt-get check` all succeed;
- Python package metadata and the ROS 2 CLI load successfully;
- ROS 2 Humble reports correctly and the ROSOrin graph is running;
- `colcon list` identifies all 24 vendor workspace packages when invoked from
  `/home/ubuntu/ros2_ws`;
- NVMe SMART remains healthy with no media or controller-log errors.

Remaining platform work before LAKSA integration:

- system time is incorrect (June 2, 2026, Asia/Shanghai), the RTC is at 1970,
  and NTP is active but has received no packets because the Jetson currently
  has no Internet-facing interface;
- the 65 GiB root partition is 88% full while approximately 399 GiB remains
  unallocated at the end of the 500 GB NVMe; GPT also reports unused device
  capacity;
- the vendor workspace is still not a Git repository;
- `twist_mux` is available from APT but not installed;
- `micro_ros_agent` is not installed and is not offered by the currently
  queried binary package set;
- Xbox, ESP32, LiDAR and camera devices were not connected during this audit.

The running ROSOrin graph confirms the vendor bringup starts automatically.
Core nodes include open-loop `odom_publisher`, `robot_localization`, the vendor
`joystick_control`, robot description, rosbridge/web video, and application
nodes. The camera image topic has a subscriber but no publisher, and no laser
scan topic is present, consistent with disconnected sensors. The legacy motor
controller must be excluded from the LAKSA command path before testing motion.

## Historical: failed reinstallation verification

The Jetson was reflashed/reinstalled and audited again. The replacement did
not produce a healthy root filesystem:

- the same EXT4 directory inode `50575` immediately reports checksum errors;
- root is still the same 65 GiB layout with approximately 54 GiB used;
- the ROSOrin workspace and its build artifacts are still present unchanged;
- `/etc/apt/apt.conf.d/00aptitude` remains invalid;
- `/var/lib/dpkg/status` now demonstrably contains invalid binary data;
- the filesystem reports `clean with errors` and requests `e2fsck -D`;
- the root filesystem creation/mount timestamps are recorded as 1970.

The system is booting `/dev/nvme0n1p1` through PARTUUID
`7e601f05-7305-42c0-afad-85b790a82e91`. These observations strongly indicate
that either only the Jetson boot firmware was reflashed while retaining the
old NVMe rootfs, or the vendor image itself contains the damaged filesystem.
The exact image URL/version and flashing procedure are needed to distinguish
the two.

## Historical: filesystem integrity blocker

The root filesystem on `/dev/nvme0n1p1` is actively reporting EXT4 directory
checksum errors and requests an offline `e2fsck -D`. Observable consequences:

- files under the Python user installation return `EBADMSG` (`Bad message`);
- `ros2` initially fails while reading damaged Torch package metadata;
- even with the user site disabled, ROS CLI processes terminate with
  `Illegal instruction`;
- `/etc/apt/apt.conf.d/00aptitude` contains invalid/corrupted content;
- kernel logs show repeated checksum failures in multiple directory inodes.

Do not install packages, run `colcon build`, or edit the ROS workspace until
the filesystem is backed up and repaired offline. Never run a repairing
`e2fsck` against the currently mounted root filesystem.

NVMe SMART does not report media errors or wear: 0 critical warnings, 0 media
errors, 0 error-log entries, 100% spare and 0% life used. It does report three
unsafe shutdowns. Healthy SMART does not invalidate the EXT4 corruption seen
by the kernel.

Storage layout:

- drive: Sandisk Optimus 5100 500 GB, approximately 466 GiB visible;
- root partition: 65 GiB EXT4, mounted read/write;
- root usage: 54 GiB used, 7.6 GiB free, 88% full;
- most of the NVMe capacity is not allocated to the root partition.

## Platform inventory

- NVIDIA Jetson Orin, `aarch64`;
- Ubuntu 22.04.5 LTS;
- NVIDIA Tegra kernel `5.15.148-tegra`;
- NVIDIA driver `540.4.0`, CUDA compatibility reported as 12.6;
- ROS 2 Humble under `/opt/ros/humble`;
- ROSOrin workspace: `/home/ubuntu/ros2_ws`;
- workspace is not a Git repository;
- source/build/install/log sizes: approximately 492/68/18/15 MiB.

Installed ROS packages include `joy`, `teleop_twist_joy`,
`robot_localization`, `slam_toolbox`, the vendor `bringup` and `controller`.
`micro_ros_agent` was not found in the inspected environment.

Connected hardware during inspection:

- Realtek USB 2.0/3.0 hubs;
- IMC Networks Bluetooth radio;
- Silicon Labs CP210x UART bridge at `/dev/ttyUSB0`;
- no `/dev/ttyACM*`, joystick device or video device was visible;
- therefore the ESP32 native USB, Xbox controller and camera were not connected.

## Existing ROSOrin software

The vendor `bringup.launch.py` starts all of these together:

- legacy controller and open-loop odometry;
- depth camera;
- LiDAR;
- rosbridge websocket and web video server;
- application nodes;
- vendor joystick controller;
- initial servo pose.

The launch system depends on global environment variables such as
`MACHINE_TYPE`, `LIDAR_TYPE`, `DEPTH_CAMERA_TYPE` and `need_compile`. This is
fragile and should become explicit launch arguments/parameter files.

### Existing command and odometry path

The vendor odometry/controller node subscribes simultaneously to:

- `controller/cmd_vel`;
- `app/cmd_vel`;
- `cmd_vel`.

For `ROSOrin_Acker`, it computes steering and wheel commands, then publishes
vendor-specific motor and PWM-servo messages to `ros_robot_controller`. Its
`odom_raw` is integrated from the commanded velocity at 50 Hz, not from
measured wheel motion. That must not remain the authoritative odometry after
the ESP32/VESC integration.

The existing joystick node is also vendor-specific. It consumes
`ros_robot_controller/joy`, not Linux `/joy`, and publishes
`controller/cmd_vel`. For an Xbox connected to the Jetson, use the installed
community `joy/game_controller_node` and `teleop_twist_joy` instead.

### Existing sensor support

LiDAR launch code has branches/configuration for LD19, MS200, SCLIDAR, A1, G4
and other vendor variants. It normalizes output through `scan_raw` and `scan`,
with `lidar_frame` as the expected frame.

Camera launch code supports `ascamera`, `aurora`, and a USB-camera fallback.
The fallback currently creates a static transform involving
`ascamera_camera_link_0` and `depth_cam_color_frame`, which must be replaced
with the actual optical frames and measured mounting transform of the new
camera.

The robot description already contains an Ackermann model and uses the frame
chain around `map`, `odom`, `base_footprint`, `imu_link`, camera and LiDAR.

## Proposed LAKSA architecture

```text
Xbox -> joy -> teleop_twist_joy -> /cmd_vel_joy --+
                                                   +-> twist_mux -> /cmd_vel -> ESP32
Nav2 ------------------------------> /cmd_vel_nav --+

ESP32 /laksa/vesc/state ----> wheel odometry adapter --+
ESP32 /laksa/imu/data ----------------------------------+-> robot_localization EKF
ESP32 /laksa/imu/mag -----------------------------------+      -> /odom + odom TF

LiDAR driver  -> /scan ---------------------------------------> SLAM/Nav2
Camera driver -> image + camera_info --------------------------> vision/RTAB-Map
```

Create a Jetson overlay workspace with these responsibilities:

1. `laksa_interfaces`: the same generated interface definitions used by the
   ESP32 firmware.
2. `laksa_bringup`: launch the serial Agent, controller input mux, sensors,
   transforms and state estimation using explicit arguments.
3. A small odometry adapter: convert measured wheel speed plus measured
   steering angle into `nav_msgs/Odometry`. Use `robot_localization` for fusion
   instead of writing a custom EKF.
4. Stable udev names such as `/dev/laksa_esp32` and `/dev/laksa_lidar`, based on
   USB identity/serial rather than `ttyACM0` or `ttyUSB0` enumeration order.

The legacy controller must not remain subscribed to the final `/cmd_vel` while
micro-ROS is active, otherwise two hardware backends can receive the same
motion request. It can be disabled entirely or launched only for peripherals
that are still physically required.

Use `twist_mux` so Xbox commands, Nav2 commands and emergency locks have one
explicit arbitration point. The ESP32's 500 ms watchdog remains the final
hardware-side stop layer.

## Geometry and units that must be reconciled

ROSOrin currently hardcodes approximately:

- wheelbase: 0.17706 m;
- track width: 0.17165 m;
- wheel diameter: 0.085 m.

The current ESP32 defaults are placeholders of 0.250 m wheelbase and 0.100 m
wheel diameter. Before motion/odometry testing, measure the real vehicle and
make one version-controlled YAML/Xacro source authoritative. Also record motor
pole pairs, gear reduction, VESC direction sign, maximum steering angle and
servo calibration.

## Safe implementation sequence

1. Back up readable source/configuration to another machine or disk.
2. Boot from recovery/external media and repair `/dev/nvme0n1p1` while it is
   unmounted; then verify kernel logs and affected files.
3. Repair the malformed APT configuration and reinstall corrupted Python/ROS
   packages only after filesystem integrity is restored.
4. Put `/home/ubuntu/ros2_ws/src` under Git before changing ROSOrin.
5. Install/build `micro_ros_agent`, `laksa_interfaces`, `twist_mux`, and the
   LAKSA bringup/odometry packages.
6. Bench-test Xbox -> `/cmd_vel` -> ESP32 with driven wheels raised and legacy
   motor output disabled.
7. Validate measured VESC speed and steering, then EKF odometry.
8. Add the exact LiDAR and camera drivers, topic remaps, QoS and calibrated TFs.
9. Connect Nav2 only after manual control and all stop paths are verified.

The exact replacement LiDAR and camera model numbers/interfaces are still
required before selecting their drivers and launch parameters.
