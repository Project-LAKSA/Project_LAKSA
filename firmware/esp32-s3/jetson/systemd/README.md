# LAKSA base services on the Jetson

These services start at boot regardless of driving mode:

- `laksa-wifi-reconnect.service` asks NetworkManager to reconnect the existing
  `LAKSA-phone-hotspot` profile whenever Wi-Fi drops. The password remains in
  NetworkManager and is not stored in this repository.
- `laksa-xbox-reconnect.service` keeps the previously paired and trusted Xbox
  controller connected. It retries without blocking other services while the
  controller is off.
- `laksa-micro-ros-agent.service` waits for `/dev/laksa_microros` and runs the
  official Humble Agent. It recovers after the ESP32 is disconnected and
  reconnected.
- `laksa-zed-camera.service` waits for the ZED 2i USB device and publishes
  RGB, registered depth, reduced point clouds, visual odometry, and camera IMU
  data. It deliberately does not publish the vehicle-to-camera transform;
  3-D fusion remains calibration-locked until `zed_mount.yaml` is measured.
- `laksa-lidar-mapping.service` waits for `/dev/laksa_lidar`, starts the
  RPLIDAR A2M12 at 256000 baud, keeps `/scan` alive, estimates planar odometry
  with RF2O, and launches asynchronous SLAM Toolbox in mapping mode.
- `laksa-control-navigation.service` starts the single drive-command authority,
  Nav2 MPPI, MRTSP frontier exploration, and the web dashboard. VESC data is
  retained as longitudinal-speed and motor telemetry; it is not used to infer
  an unmeasured steering angle or publish the odometry TF. The service boots in
  manual mode and rejects autonomy until its health
  gate reports `READY`. Hold A for three seconds to explore, X returns to
  manual, and B requests active VESC braking.

`xbox_drive_node` is not started automatically. Running it alongside the drive
supervisor would create two command publishers for the same actuators.

The generic ROSOrin `start_app_node.service` must remain disabled while LAKSA
is active. That vendor bringup starts its own localization, joystick, servo,
vision, and web nodes in ROS domain 0. The packages remain installed for reuse,
but running the complete vendor graph beside LAKSA wastes compute and creates
ambiguous control and TF authorities:

```bash
sudo systemctl disable --now start_app_node.service
```

The ROS services use `ROS_LOCALHOST_ONLY=1` because all ROS 2 computation and
the serial micro-ROS Agent run on the Jetson. Dashboard HTTP remains reachable
over Wi-Fi, while DDS discovery no longer breaks when the Wi-Fi interface
changes address or reconnects.

## Installation

```bash
jetson/scripts/install-rf2o-laser-odometry
jetson/scripts/install-frontier-exploration
sudo install -D -m 0755 jetson/scripts/laksa-wifi-reconnect /usr/local/lib/laksa/laksa-wifi-reconnect
sudo install -D -m 0755 jetson/scripts/laksa-xbox-reconnect /usr/local/lib/laksa/laksa-xbox-reconnect
sudo install -D -m 0755 jetson/scripts/laksa-micro-ros-agent /usr/local/lib/laksa/laksa-micro-ros-agent
sudo install -D -m 0755 jetson/scripts/laksa-zed-camera /usr/local/lib/laksa/laksa-zed-camera
sudo install -D -m 0755 jetson/scripts/laksa-lidar-mapping /usr/local/lib/laksa/laksa-lidar-mapping
sudo install -D -m 0755 jetson/scripts/laksa-control-navigation /usr/local/lib/laksa/laksa-control-navigation
sudo install -m 0644 jetson/systemd/laksa-wifi-reconnect.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-xbox-reconnect.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-micro-ros-agent.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-zed-camera.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-lidar-mapping.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-control-navigation.service /etc/systemd/system/
sudo install -D -m 0644 jetson/systemd/laksa-mapping.conf /etc/laksa/mapping.conf
sudo install -m 0644 jetson/udev/99-laksa-devices.rules /etc/udev/rules.d/
sudo systemctl daemon-reload
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl enable --now laksa-wifi-reconnect.service laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-zed-camera.service laksa-lidar-mapping.service laksa-control-navigation.service
```

## Diagnostics

```bash
systemctl status laksa-wifi-reconnect.service laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-zed-camera.service laksa-lidar-mapping.service laksa-control-navigation.service
journalctl -u laksa-wifi-reconnect.service -u laksa-xbox-reconnect.service -u laksa-micro-ros-agent.service -u laksa-zed-camera.service -u laksa-lidar-mapping.service -u laksa-control-navigation.service -f
```
