#!/usr/bin/env python3

"""Monitor ZED image and IMU streams without spawning ROS CLI processes."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class ZedHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("laksa_zed_health_monitor")
        self.declare_parameter("startup_timeout_sec", 50.0)
        self.declare_parameter("stream_timeout_sec", 12.0)
        self._startup_timeout = float(
            self.get_parameter("startup_timeout_sec").value
        )
        self._stream_timeout = float(
            self.get_parameter("stream_timeout_sec").value
        )
        self._started = time.monotonic()
        self._last_image = 0.0
        self._last_imu = 0.0
        self._ready = False
        self.failed = False
        self.create_subscription(
            Image,
            "/zed/zed_node/rgb/color/rect/image",
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/zed/zed_node/imu/data",
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._check)

    def _image_callback(self, _message: Image) -> None:
        self._last_image = time.monotonic()

    def _imu_callback(self, _message: Imu) -> None:
        self._last_imu = time.monotonic()

    def _check(self) -> None:
        now = time.monotonic()
        both_active = self._last_image > 0.0 and self._last_imu > 0.0
        if not self._ready:
            if both_active:
                self._ready = True
                self.get_logger().info("ZED 2i image and IMU streams are active")
                return
            if now - self._started <= self._startup_timeout:
                return
            self._fail("ZED streams did not become healthy before startup timeout")
            return
        if (
            now - self._last_image > self._stream_timeout
            or now - self._last_imu > self._stream_timeout
        ):
            self._fail("ZED image or IMU stream became stale")

    def _fail(self, reason: str) -> None:
        self.failed = True
        self.get_logger().error(reason)
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        failed = node.failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
