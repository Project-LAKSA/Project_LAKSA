# LAKSA Jetson bringup

This package uses the standard ROS 2 `joy` and `joy_teleop` packages. It does
not implement a joystick driver or a USB protocol. The ESP32 native USB CDC
interface is owned by the standard micro-ROS Agent.

## Manual command path

```text
Xbox -> game_controller_node -> /joy -> joy_teleop
     -> /laksa/command (laksa_interfaces/DriveCommand)
     -> micro-ROS Agent -> ESP32 watchdog/limits -> actuators
```

Hold the Xbox right bumper while commanding traction with the left stick Y
axis and steering with the right stick X axis. Releasing the bumper stops
publishing; the ESP32 command watchdog stops and centers the vehicle within
500 ms.

## Build in a Jetson overlay

Copy `laksa_interfaces` and `laksa_bringup` into the workspace, then build:

```bash
mkdir -p ~/laksa_ws/src
cp -a extra_ros_packages/laksa_interfaces ~/laksa_ws/src/
cp -a jetson/laksa_bringup ~/laksa_ws/src/
cd ~/laksa_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Start the micro-ROS Agent in one terminal and the controller launch in another:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/laksa_microros -b 115200 -v4
ros2 launch laksa_bringup xbox_drive.launch.py
```

Before permitting motion, verify `/laksa_esp32`, `/laksa/command`, and
`/laksa/vesc/state`. Perform the first nonzero test with the driven wheels
raised and the vendor ROSOrin bringup stopped.
