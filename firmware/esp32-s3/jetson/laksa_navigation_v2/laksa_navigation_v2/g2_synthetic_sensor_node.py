"""ROS 2 publisher for G2 fixtures. It is strictly test-only and command-free."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .g2_synthetic_inputs import generate_case, speed_twist_covariance, vio_pose_covariance


def _stamp(message, now, offset_sec: float = 0.0) -> None:
    nanoseconds = now.nanoseconds + int(offset_sec * 1_000_000_000)
    message.header.stamp.sec = nanoseconds // 1_000_000_000
    message.header.stamp.nanosec = nanoseconds % 1_000_000_000


class G2SyntheticSensorNode(Node):
    def __init__(self) -> None:
        super().__init__("g2_test_only_synthetic_inputs")
        self.declare_parameter("scenario", "G2_S001_STATIONARY")
        self.declare_parameter("period_sec", 0.05)
        self._measurements = generate_case(str(self.get_parameter("scenario").value))
        self._index = 0
        self._vio = self.create_publisher(Odometry, "/laksa/vio/odom", 20)
        self._speed = self.create_publisher(TwistWithCovarianceStamped, "/laksa/vehicle/speed", 20)
        self._timer = self.create_timer(float(self.get_parameter("period_sec").value), self._tick)

    def _tick(self) -> None:
        if self._index >= len(self._measurements):
            self._timer.cancel()
            return
        sample = self._measurements[self._index]
        now = self.get_clock().now()
        if sample.vio_available:
            message = Odometry()
            _stamp(message, now, sample.vio_stamp_sec - sample.truth.stamp_sec)
            message.header.frame_id = "odom"
            message.child_frame_id = "zed_camera_link"
            message.pose.pose.position.x = sample.vio_x_m
            message.pose.pose.position.y = sample.vio_y_m
            message.pose.pose.orientation.z = math.sin(sample.vio_yaw_rad / 2.0)
            message.pose.pose.orientation.w = math.cos(sample.vio_yaw_rad / 2.0)
            message.pose.covariance = vio_pose_covariance()
            self._vio.publish(message)
        if sample.speed_available:
            message = TwistWithCovarianceStamped()
            _stamp(message, now, sample.speed_stamp_sec - sample.truth.stamp_sec)
            message.header.frame_id = "base_footprint"
            message.twist.twist.linear.x = sample.speed_mps
            message.twist.covariance = speed_twist_covariance()
            self._speed.publish(message)
        self._index += 1


def main() -> None:
    rclpy.init()
    node = G2SyntheticSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
