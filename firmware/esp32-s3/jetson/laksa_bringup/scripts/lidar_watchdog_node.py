#!/usr/bin/env python3

"""Keep the SLAMTEC motor scanning when its initial motor command is missed."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class LidarWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("lidar_watchdog")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("startup_grace_sec", 2.0)
        self.declare_parameter("scan_timeout_sec", 2.0)
        self.declare_parameter("retry_period_sec", 2.0)
        self.declare_parameter("max_start_attempts", 3)

        scan_topic = str(self.get_parameter("scan_topic").value)
        self._startup_grace = float(self.get_parameter("startup_grace_sec").value)
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        retry_period = float(self.get_parameter("retry_period_sec").value)
        self._max_start_attempts = int(
            self.get_parameter("max_start_attempts").value
        )

        now = time.monotonic()
        self._started_at = now
        self._last_scan_at = None
        self._request = None
        self._reported_scanning = False
        self._start_attempts = 0

        self._start_client = self.create_client(Empty, "/start_motor")
        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.create_timer(retry_period, self._check_scan)
        self.get_logger().info(f"Watching {scan_topic} for RPLIDAR scans")

    def _scan_callback(self, _message: LaserScan) -> None:
        self._last_scan_at = time.monotonic()
        self._start_attempts = 0
        if not self._reported_scanning:
            self.get_logger().info("RPLIDAR scan stream is active")
            self._reported_scanning = True

    def _check_scan(self) -> None:
        now = time.monotonic()
        if self._last_scan_at is None:
            stream_stale = now - self._started_at >= self._startup_grace
        else:
            stream_stale = now - self._last_scan_at >= self._scan_timeout

        if not stream_stale:
            return

        if self._request is not None and not self._request.done():
            return

        self._reported_scanning = False
        if self._start_attempts >= self._max_start_attempts:
            self.get_logger().error(
                "Scan stream did not recover; requesting a full stack restart"
            )
            raise SystemExit(1)

        if not self._start_client.service_is_ready():
            self.get_logger().warning("Waiting for the SLAMTEC /start_motor service")
            return

        self.get_logger().warning("No recent scans; requesting RPLIDAR motor start")
        self._start_attempts += 1
        self._request = self._start_client.call_async(Empty.Request())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
