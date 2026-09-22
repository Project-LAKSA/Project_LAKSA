"""Observational VESC telemetry adapter; it has no actuator capability."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from laksa_interfaces.msg import VescState
from rclpy.node import Node

from .vehicle_speed_adapter_contract import measured_speed_is_usable


class VehicleSpeedAdapter(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_speed_adapter")
        self.declare_parameter("variance_mps2", -1.0)
        self._variance = float(self.get_parameter("variance_mps2").value)
        self._publisher = self.create_publisher(TwistWithCovarianceStamped, "/laksa/vehicle/speed", 10)
        self.create_subscription(VescState, "/laksa/vesc/state", self._callback, 10)
        if self._variance <= 0.0:
            self.get_logger().warn("physical VESC speed variance is uncalibrated; adapter will not publish")

    def _callback(self, state: VescState) -> None:
        speed = float(state.vehicle_linear_velocity_mps)
        if not measured_speed_is_usable(bool(state.telemetry_fresh), speed, self._variance):
            return
        message = TwistWithCovarianceStamped()
        message.header.stamp = state.stamp
        message.header.frame_id = "base_footprint"
        message.twist.twist.linear.x = speed
        # A covariance is a row-major matrix, not a vector of variances.
        # Only Vx's physical variance is known to this adapter; non-fused
        # diagonal entries stay conservative and every off-diagonal is zero.
        message.twist.covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            message.twist.covariance[index] = 1.0
        message.twist.covariance[0] = self._variance
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = VehicleSpeedAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
