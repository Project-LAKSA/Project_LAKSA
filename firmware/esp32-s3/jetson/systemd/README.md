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
- `laksa-lidar.service` keeps the RPLIDAR A2M12 driver, self-filter, watchdog,
  and `/scan` stream available without starting RF2O or SLAM Toolbox.
- `laksa-control-navigation.service` starts only the Xbox controller and the
  existing drive supervisor by default. Legacy Nav2, LiDAR cruise, and the old
  mission dashboard require the explicit `LAKSA_ENABLE_AUTONOMY=true` opt-in.
- `laksa-mapping-cockpit.service` is the boot-time PROJECT LAKSA Field Lab. Its
  session manager is the sole owner of ZED2i, rgbd_sync, and RTAB-Map processes.
- `laksa-planning-preview.service` is a legacy, deliberately disabled dry-run
  unit. Canonical saved-map navigation is owned by Field Lab after TAKE
  SNAPSHOT; this unit must not run concurrently with it.
- `laksa-zed-camera.service` and `laksa-lidar-mapping.service` remain available
  for explicit legacy diagnostics, but have no boot install target.

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

All host-side ROS services require `/etc/laksa/ros-runtime.env`. It selects the
qualified CycloneDDS RMW and one versioned Cyclone configuration so child
processes launched by Field Lab cannot silently fall back to FastDDS.

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
sudo install -D -m 0755 jetson/scripts/laksa-mapping-cockpit /usr/local/lib/laksa/laksa-mapping-cockpit
sudo install -D -m 0755 jetson/scripts/laksa-planning-preview /usr/local/lib/laksa/laksa-planning-preview
sudo install -m 0644 jetson/systemd/laksa-wifi-reconnect.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-xbox-reconnect.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-micro-ros-agent.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-zed-camera.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-lidar-mapping.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-control-navigation.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-lidar.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-mapping-cockpit.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-planning-preview.service /etc/systemd/system/
sudo install -D -m 0644 jetson/systemd/laksa-ros-runtime.env /etc/laksa/ros-runtime.env
sudo install -D -m 0644 jetson/systemd/cyclonedds.xml /etc/laksa/cyclonedds.xml
sudo install -D -m 0644 jetson/systemd/laksa-mapping.conf /etc/laksa/mapping.conf
sudo install -m 0644 jetson/udev/99-laksa-devices.rules /etc/udev/rules.d/
sudo systemctl daemon-reload
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl disable --now laksa-zed-camera.service laksa-lidar-mapping.service
sudo systemctl disable --now laksa-planning-preview.service
sudo systemctl enable --now laksa-wifi-reconnect.service laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-lidar.service laksa-control-navigation.service laksa-mapping-cockpit.service
```

## Diagnostics

```bash
systemctl status laksa-wifi-reconnect.service laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-lidar.service laksa-control-navigation.service laksa-mapping-cockpit.service
journalctl -u laksa-micro-ros-agent.service -u laksa-lidar.service -u laksa-control-navigation.service -u laksa-mapping-cockpit.service -f
```
