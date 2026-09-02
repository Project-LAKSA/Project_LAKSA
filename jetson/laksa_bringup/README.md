# LAKSA Jetson bringup

This package uses the standard ROS 2 `joy`, Nav2, and SLAM Toolbox packages.
It does not implement a joystick driver or a USB protocol. The ESP32 native
USB CDC interface is owned by the standard micro-ROS Agent.

## Manual command path

```text
Xbox -> game_controller_node -> /joy -> drive_supervisor
     -> /laksa/command (laksa_interfaces/DriveCommand)
     -> /laksa/brake (std_msgs/Bool, dedicated safety channel)
     -> micro-ROS Agent -> ESP32 watchdog/limits -> actuators
```

The left stick Y axis selects forward or reverse after crossing its deadzone;
its magnitude does not scale speed. The dashboard selects a fixed manual VESC
setpoint from 900, 1300, 1500, 2000, or 3000 eRPM. The right stick X axis
commands steering. B latches active VESC current braking
and cancels autonomy; only Y rearms the vehicle, always into manual mode. X
returns to manual, and holding A for three seconds enables autonomous frontier
exploration. Loss of Xbox or ESP32 telemetry also requests active braking.

## Web mission control

`laksa_system.launch.py` serves the dashboard on TCP port 8088. From a device
on the same Wi-Fi network open `http://JETSON_IP:8088`. It displays the live
SLAM map, pose, IMU, VESC telemetry, currents, temperatures, command, and mode.
Drag on the map from a destination toward the desired heading to submit a Nav2
goal. The dashboard changes the supervisor to goal-navigation mode; Xbox X or
the dashboard cancel button returns control to manual. It also exposes the
mission state, exploration status, autonomous speed limit, planned path, Home,
Go Home, Navigate To Point, and a confirmed Reset Map action. Frontier
exploration and dashboard goals are mutually exclusive.

Nav2 uses Hybrid-A* with the Reeds-Shepp motion model, so planned paths may
contain forward and reverse Ackermann segments. The MPPI controller evaluates
collision-checked Ackermann trajectories and honors planner direction-change
cusps; reverse is a planned maneuver rather than a blind backup behavior.
Exploration is capped symmetrically at 1000 eRPM. During point and Home
navigation, MPPI varies speed up to the 1500 eRPM ceiling.

Frontier selection comes from the community `frontier_exploration_ros2`
package pinned by `jetson/scripts/install-frontier-exploration`. Its bounded
dynamic-programming MRTSP search evaluates a short route of frontier goals
instead of repeatedly choosing only the nearest edge. LAKSA starts it in cold
idle, disables active-goal preemption, and temporarily suppresses repeatedly
blocked regions. This gives an Ackermann maneuver time to complete and avoids
retry loops at an unreachable frontier.

The upstream one-snapshot completion event is disabled for live SLAM. A brief
empty frontier snapshot is not sufficient evidence that the environment is
fully mapped, so exploration continues until the operator presses Xbox X or B.
This deliberately favors a controllable demo over a false early completion.

The planner and controller use a conservative 1.09 m minimum turning radius,
derived from the measured 0.324 m wheelbase and 16.5-degree right road-wheel
limit. The supervisor and odometry compensate for the measured asymmetric
left/right steering endpoints. MPPI runs at 10 Hz with 900 sampled trajectories
to leave compute budget for SLAM and LiDAR processing on the Orin Nano.

The drive supervisor requires fresh ESP32,
VESC, odometry, LiDAR, map, and `map -> base` transform data before accepting
any autonomous request. The dashboard exposes this as `Autonomy: READY` or a
specific failure reason.

Reset Map validates the exact PID, owner, and systemd cgroup of the existing
`laksa-lidar-mapping.service`, then terminates only that unprivileged SLAM
process. Its existing `Restart=always` policy starts a fresh mapping session.
The dashboard also resets wheel odometry, Home, goals, and mission state, then
clears both Nav2 costmaps after the first map of the new session arrives. No
root permission is exposed to ROS or HTTP. Autonomous movement still requires
Start Exploration, Go Home, or a confirmed map destination.

## Build in a Jetson overlay

Install the pinned exploration dependency, copy `laksa_interfaces` and
`laksa_bringup` into the workspace, then build:

```bash
mkdir -p ~/laksa_ws/src
jetson/scripts/install-frontier-exploration
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
