# Jetson ROSOrin integration

## Architecture for the first test

```text
Xbox Series (Bluetooth)
  -> joy/game_controller_node
  -> /joy (sensor_msgs/Joy)
  -> joy_teleop/joy_teleop
  -> /laksa/command (laksa_interfaces/DriveCommand)
  -> micro_ros_agent (USB CDC)
  -> ESP32-S3
  -> m/s to eRPM conversion
  -> VESC
```

No custom joystick or serial protocol is needed. `joy_teleop` supports arbitrary
ROS interface types, so the manual controller can command vehicle speed in m/s
and front-wheel steering angle in radians directly. Autonomous controllers keep
using standard `geometry_msgs/Twist` on `/cmd_vel`; the ESP32 performs the
Ackermann conversion for that path.

The ESP32 command watchdog stops traction and centers steering if commands are
absent for 500 ms. The joystick driver repeats its state at 20 Hz, so an
enabled, unchanged stick still refreshes the command.

## Network and repository layout

During provisioning, connect `wlan0` to a trusted Wi-Fi network or phone
hotspot as a client. SSH over the Jetson USB gadget remains available at
`192.168.55.1`, even when the ROSOrin access-point profile is disconnected.
Never place Wi-Fi passwords in the repository.

Use a dedicated `~/laksa_ws` overlay instead of modifying the vendor
`~/ros2_ws` in place. The repository contains the reproducible source under
`firmware/esp32-s3/jetson` and `firmware/esp32-s3/extra_ros_packages`; the
deployed workspace is a build artifact. Clone the repository after Internet is
available and the current integration branch has been committed and pushed.

## Information needed from ROSOrin

Provide the URL and exact version of the ROSOrin image/source plus the output
of the following commands. Run the ROS graph normally before the final block.
Do not include passwords, tokens, Wi-Fi credentials or private keys.

```bash
cat /etc/os-release
uname -a
dpkg-query -W nvidia-l4t-core nvidia-jetpack 2>/dev/null

source /opt/ros/humble/setup.bash
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION
ros2 doctor --report
for package in micro_ros_agent joy teleop_twist_joy; do
  ros2 pkg prefix "$package" 2>&1
done

ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 action list -t
```

Also provide:

- the ROSOrin workspace or repository, including its launch, URDF/Xacro,
  parameters and Docker Compose files;
- whether ROS runs natively, in Docker, through systemd, or a combination;
- exact current and replacement LiDAR models, physical interface and driver;
- exact current and replacement camera models, physical interface and driver;
- output topic names and frame IDs from both replacement sensor drivers;
- camera intrinsics and LiDAR/camera/base_link extrinsics if already measured.

Those details determine whether replacing each sensor is only a topic/frame
remap or also requires a driver, message, QoS, timestamp or URDF change.

## Pair and inspect the Xbox controller

Install the official binary packages rather than building a controller driver:

```bash
sudo apt update
sudo apt install ros-humble-joy ros-humble-joy-teleop
```

Pair the controller in Ubuntu Bluetooth settings. Then verify that SDL sees it:

```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_enumerate_devices
ros2 run joy game_controller_node --ros-args \
  -p device_id:=0 -p deadzone:=0.10 -p autorepeat_rate:=20.0
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joy
```

For `game_controller_node`, the SDL mapping used by the supplied file is: left
stick Y = axis 1, right stick X = axis 2 and right bumper = button 10. Confirm
the actual controller fields with `/joy` before allowing the wheels to move;
reverse an axis scale if the physical direction is inverted.

## Start the ESP32 transport

Build `laksa_interfaces` and start the Humble micro-ROS Agent as described in
`MICRO_ROS.md`. Confirm the endpoint before enabling traction:

The ESP32-S3 uses two independent USB controllers here. The upper connector
exposes USB-JTAG/serial for flashing; the lower/native connector exposes
TinyUSB CDC for micro-ROS. Both may be connected simultaneously, but the Agent
must use `/dev/laksa_microros`, never the programming port. Install
`jetson/udev/99-laksa-devices.rules` for stable `/dev/laksa_microros` and
`/dev/laksa_lidar` names.

```bash
ros2 node list
ros2 topic info /laksa/command -v
ros2 topic echo /laksa/vesc/state
```

The node `/laksa_esp32` must appear and `/laksa/command` must show an ESP32
subscription.

## Run the traction-only test

First raise the driven wheels and be ready to disconnect VESC power. Keep the
Agent and `game_controller_node` running. Stop the vendor bringup so its
open-loop controller cannot also command the actuators, then start LAKSA:

```bash
sudo systemctl stop start_app_node.service
source /opt/ros/humble/setup.bash
source ~/laksa_ws/install/setup.bash
ros2 launch laksa_bringup xbox_drive.launch.py
```

Observe the outgoing command and measured traction speed in two other shells:

```bash
ros2 topic echo /laksa/command
ros2 topic echo /laksa/vesc/state
```

Hold the right bumper, move the left stick slowly for traction and use the right
stick for steering. Releasing the bumper stops refreshing commands; the ESP32
watchdog commands zero and centers steering within 500 ms. Killing the joystick
node, Agent, or unplugging USB has the same safe result.

`/laksa/command.speed_mps` and `/cmd_vel.linear.x` are expressed in vehicle m/s,
not motor eRPM or motor rad/s. The ESP32 converts them using the configured pole
pairs, gear ratio and wheel diameter. Nav2 and autonomous controllers therefore
remain on the standard ROS velocity interface while manual steering retains a
direct, unambiguous angle command.

Restore the vendor application after the isolated test:

```bash
sudo systemctl start start_app_node.service
```
