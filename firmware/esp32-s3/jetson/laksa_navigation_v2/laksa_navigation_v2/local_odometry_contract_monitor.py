"""Read-only local-odometry contract monitor for G2.1.

This is not a motion arbiter. It publishes standard diagnostics only, allowing
G7 to consume the fail-closed LOCAL_ODOMETRY_HEALTHY semantics later.
"""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class LocalOdometryContractMonitor(Node):
    def __init__(self) -> None:
        super().__init__("local_odometry_contract_monitor")
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("output_timeout_sec", 0.5)
        self._input_timeout = float(self.get_parameter("input_timeout_sec").value)
        self._output_timeout = float(self.get_parameter("output_timeout_sec").value)
        self._vio_ns: int | None = None
        self._speed_ns: int | None = None
        self._output_ns: int | None = None
        self._last_output_stamp: int | None = None
        self._frame_fault: str | None = None
        self._numeric_fault = False
        self._tf_seen = False
        self._static_zed_seen = False
        self._publisher = self.create_publisher(DiagnosticArray, "/laksa/odometry/local/diagnostics", 10)
        self.create_subscription(Odometry, "/laksa/vio/odom", self._vio, 20)
        self.create_subscription(TwistWithCovarianceStamped, "/laksa/vehicle/speed", self._speed, 20)
        self.create_subscription(Odometry, "/laksa/odometry/local", self._output, 20)
        self.create_subscription(TFMessage, "/tf", self._tf, 20)
        self.create_subscription(TFMessage, "/tf_static", self._tf_static, 10)
        self.create_timer(0.1, self._publish)

    @staticmethod
    def _stamp_ns(message) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    @staticmethod
    def _finite_odometry(message: Odometry) -> bool:
        values = [message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z,
                  message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z,
                  message.pose.pose.orientation.w, *message.pose.covariance, *message.twist.covariance]
        return all(math.isfinite(value) for value in values)

    def _vio(self, message: Odometry) -> None:
        if message.header.frame_id != "odom" or message.child_frame_id != "zed_camera_link":
            self._frame_fault = "LOCAL_ODOMETRY_INPUT_INVALID: raw ZED frame contract"
        elif self._stamp_ns(message) <= 0 or not self._finite_odometry(message):
            self._frame_fault = "LOCAL_ODOMETRY_INPUT_INVALID: invalid raw ZED stamp/data"
        else:
            self._vio_ns = self.get_clock().now().nanoseconds

    def _output(self, message: Odometry) -> None:
        stamp = self._stamp_ns(message)
        if message.header.frame_id != "odom" or message.child_frame_id != "base_footprint":
            self._frame_fault = "LOCAL_ODOMETRY_OUTPUT_INVALID: frame contract"
        if self._last_output_stamp is not None and stamp < self._last_output_stamp:
            self._frame_fault = "LOCAL_ODOMETRY_OUTPUT_INVALID: timestamp regression"
        self._last_output_stamp = stamp
        if not self._finite_odometry(message):
            self._numeric_fault = True
        self._output_ns = self.get_clock().now().nanoseconds

    def _speed(self, message: TwistWithCovarianceStamped) -> None:
        if message.header.frame_id != "base_footprint" or self._stamp_ns(message) <= 0 or not all(math.isfinite(value) for value in message.twist.covariance):
            self._frame_fault = "LOCAL_ODOMETRY_INPUT_INVALID: vehicle-speed contract"
        else:
            self._speed_ns = self.get_clock().now().nanoseconds

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_footprint":
                self._tf_seen = True

    def _tf_static(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "base_link" and transform.child_frame_id == "zed_camera_link":
                self._static_zed_seen = True

    def _publish(self) -> None:
        now = self.get_clock().now().nanoseconds
        age = lambda stamp: float("inf") if stamp is None else (now - stamp) / 1_000_000_000
        vio_fresh, output_fresh = age(self._vio_ns) <= self._input_timeout, age(self._output_ns) <= self._output_timeout
        healthy = vio_fresh and output_fresh and self._tf_seen and self._static_zed_seen and not self._numeric_fault and self._frame_fault is None
        reason = "OK" if healthy else (self._frame_fault or ("LOCAL_ODOMETRY_INPUT_INVALID: stale VIO" if not vio_fresh else "LOCAL_ODOMETRY_OUTPUT_INVALID: stale output/TF"))
        status = DiagnosticStatus(level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
                                  name="laksa/local_odometry", message=reason,
                                  hardware_id="laksa_navigation_v2")
        status.values = [KeyValue(key="LOCAL_ODOMETRY_HEALTHY", value=str(healthy).lower()),
                         KeyValue(key="vio_age_sec", value=f"{age(self._vio_ns):.3f}"),
                         KeyValue(key="speed_age_sec", value=f"{age(self._speed_ns):.3f}"),
                         KeyValue(key="output_age_sec", value=f"{age(self._output_ns):.3f}")]
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg(); array.status = [status]
        self._publisher.publish(array)


def main() -> None:
    rclpy.init(); node = LocalOdometryContractMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
