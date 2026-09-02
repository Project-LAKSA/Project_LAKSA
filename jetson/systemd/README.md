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
- `laksa-lidar-mapping.service` waits for `/dev/laksa_lidar`, starts the
  RPLIDAR A2M12 at 256000 baud, keeps `/scan` alive, and launches SLAM Toolbox
  in mapping mode.
- `laksa-control-navigation.service` starts the single drive-command authority,
  measured VESC Ackermann odometry, Nav2 MPPI, `explore_lite`, and the web
  dashboard. It boots in manual mode and rejects autonomy until its health
  gate reports `READY`. Hold A for three seconds to explore, X returns to
  manual, and B requests active VESC braking.

`xbox_drive_node` is not started automatically. Running it alongside the drive
supervisor would create two command publishers for the same actuators.

The ROS services use `ROS_LOCALHOST_ONLY=1` because all ROS 2 computation and
the serial micro-ROS Agent run on the Jetson. Dashboard HTTP remains reachable
over Wi-Fi, while DDS discovery no longer breaks when the Wi-Fi interface
changes address or reconnects.

## Installation

```bash
sudo install -D -m 0755 jetson/scripts/laksa-wifi-reconnect /usr/local/lib/laksa/laksa-wifi-reconnect
sudo install -D -m 0755 jetson/scripts/laksa-xbox-reconnect /usr/local/lib/laksa/laksa-xbox-reconnect
sudo install -D -m 0755 jetson/scripts/laksa-micro-ros-agent /usr/local/lib/laksa/laksa-micro-ros-agent
sudo install -D -m 0755 jetson/scripts/laksa-lidar-mapping /usr/local/lib/laksa/laksa-lidar-mapping
sudo install -D -m 0755 jetson/scripts/laksa-control-navigation /usr/local/lib/laksa/laksa-control-navigation
sudo install -m 0644 jetson/systemd/laksa-wifi-reconnect.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-xbox-reconnect.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-micro-ros-agent.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-lidar-mapping.service /etc/systemd/system/
sudo install -m 0644 jetson/systemd/laksa-control-navigation.service /etc/systemd/system/
sudo install -m 0644 jetson/udev/99-laksa-devices.rules /etc/udev/rules.d/
sudo systemctl daemon-reload
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl enable --now laksa-wifi-reconnect.service laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-lidar-mapping.service laksa-control-navigation.service
```

## Diagnostics

```bash
systemctl status laksa-wifi-reconnect.service laksa-xbox-reconnect.service laksa-micro-ros-agent.service laksa-lidar-mapping.service laksa-control-navigation.service
journalctl -u laksa-wifi-reconnect.service -u laksa-xbox-reconnect.service -u laksa-micro-ros-agent.service -u laksa-lidar-mapping.service -u laksa-control-navigation.service -f
```
