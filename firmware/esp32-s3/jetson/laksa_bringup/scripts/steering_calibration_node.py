#!/usr/bin/env python3

"""Increment and hold steering for supervised, zero-traction calibration."""

import math

import rclpy
from laksa_interfaces.msg import DriveCommand, VehicleState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


class SteeringCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("steering_calibration_node")

        self.declare_parameter("steering_axis", 0)
        self.declare_parameter("axis_to_steering_sign", -1.0)
        self.declare_parameter("deadzone", 0.20)
        self.declare_parameter("increment_rate_rad_s", 0.05)
        self.declare_parameter("max_abs_steering_rad", 0.523)
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("joy_timeout_sec", 0.50)

        self._axis = int(self.get_parameter("steering_axis").value)
        self._axis_sign = float(
            self.get_parameter("axis_to_steering_sign").value
        )
        self._deadzone = float(self.get_parameter("deadzone").value)
        self._increment_rate = float(
            self.get_parameter("increment_rate_rad_s").value
        )
        self._max_angle = float(
            self.get_parameter("max_abs_steering_rad").value
        )
        update_rate = float(self.get_parameter("update_rate_hz").value)
        self._joy_timeout_ns = int(
            float(self.get_parameter("joy_timeout_sec").value) * 1_000_000_000
        )

        finite_values = (
            self._axis_sign,
            self._deadzone,
            self._increment_rate,
            self._max_angle,
            update_rate,
        )
        if self._axis < 0 or not all(math.isfinite(value) for value in finite_values):
            raise ValueError("Calibration parameters must be finite and axis non-negative")
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        if self._axis_sign not in (-1.0, 1.0):
            raise ValueError("axis_to_steering_sign must be -1 or 1")
        if self._increment_rate <= 0.0 or self._max_angle <= 0.0:
            raise ValueError("Steering rate and limit must be positive")
        if update_rate <= 0.0 or self._joy_timeout_ns <= 0:
            raise ValueError("Update rate and joystick timeout must be positive")

        self._period_sec = 1.0 / update_rate
        self._publisher = self.create_publisher(DriveCommand, "/laksa/command", 10)
        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_subscription(
            VehicleState,
            "/laksa/state",
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(self._period_sec, self._update)

        self._last_joy_ns = 0
        self._stick = 0.0
        self._target = 0.0
        self._state_received = False
        self._armed = False
        self._last_reported_millirad = None

        self.get_logger().info(
            "ZERO-TRACTION steering calibration: center the left stick to arm"
        )

    def _state_callback(self, message: VehicleState) -> None:
        if not self._state_received:
            self._target = max(
                -self._max_angle,
                min(self._max_angle, float(message.steering_current_rad)),
            )
            self._state_received = True
            self.get_logger().info(
                f"Initialized from ESP32 steering state: {self._target:+.4f} rad"
            )

    def _joy_callback(self, message: Joy) -> None:
        if len(message.axes) <= self._axis:
            self.get_logger().error(
                f"Joy message has {len(message.axes)} axes; need axis {self._axis}",
                throttle_duration_sec=2.0,
            )
            return

        self._last_joy_ns = self.get_clock().now().nanoseconds
        self._stick = float(message.axes[self._axis])
        if (
            self._state_received
            and not self._armed
            and abs(self._stick) <= self._deadzone
        ):
            self._armed = True
            self.get_logger().info(
                "Calibration armed: rightward left-stick motion decreases ROS angle; "
                "center holds position"
            )

    def _publish_target(self) -> None:
        command = DriveCommand()
        command.speed_mps = 0.0
        command.steering_angle_rad = self._target
        command.brake = True
        self._publisher.publish(command)

        millirad = round(self._target * 1000.0)
        if millirad != self._last_reported_millirad:
            self._last_reported_millirad = millirad
            self.get_logger().info(f"Steering target: {self._target:+.4f} rad")

    def _update(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        joy_fresh = (
            self._last_joy_ns != 0
            and now_ns - self._last_joy_ns <= self._joy_timeout_ns
        )
        if not self._armed or not joy_fresh or abs(self._stick) <= self._deadzone:
            return

        magnitude = (abs(self._stick) - self._deadzone) / (1.0 - self._deadzone)
        direction = math.copysign(1.0, self._stick) * self._axis_sign
        next_target = self._target + (
            direction * magnitude * self._increment_rate * self._period_sec
        )
        next_target = max(-self._max_angle, min(self._max_angle, next_target))
        if next_target == self._target:
            return

        self._target = next_target
        self._publish_target()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SteeringCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
