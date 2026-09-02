#!/usr/bin/env python3

"""Map the LAKSA Xbox controls to a fail-safe DriveCommand stream."""

import math

import rclpy
from laksa_interfaces.msg import DriveCommand
from rclpy.node import Node
from sensor_msgs.msg import Joy


class XboxDriveNode(Node):
    def __init__(self) -> None:
        super().__init__("xbox_drive_node")

        self.declare_parameter("steering_axis", 2)
        self.declare_parameter("steering_scale_rad", -0.35)
        self.declare_parameter("trigger_axis", 5)
        self.declare_parameter("trigger_pressed_threshold", -0.50)
        self.declare_parameter("trigger_released_threshold", -0.10)
        self.declare_parameter("pressed_speed_mps", 0.6731984)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("joy_timeout_sec", 0.25)

        self._steering_axis = int(self.get_parameter("steering_axis").value)
        self._steering_scale = float(self.get_parameter("steering_scale_rad").value)
        self._trigger_axis = int(self.get_parameter("trigger_axis").value)
        self._trigger_pressed = float(
            self.get_parameter("trigger_pressed_threshold").value
        )
        self._trigger_released = float(
            self.get_parameter("trigger_released_threshold").value
        )
        self._pressed_speed = float(self.get_parameter("pressed_speed_mps").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self._joy_timeout_ns = int(
            float(self.get_parameter("joy_timeout_sec").value) * 1_000_000_000
        )

        if self._steering_axis < 0 or self._trigger_axis < 0:
            raise ValueError("Joystick axes must be non-negative")
        if not all(
            math.isfinite(value)
            for value in (
                self._steering_scale,
                self._trigger_pressed,
                self._trigger_released,
                self._pressed_speed,
                publish_rate,
            )
        ):
            raise ValueError("Xbox drive parameters must be finite")
        if publish_rate <= 0.0 or self._joy_timeout_ns <= 0:
            raise ValueError("Publish rate and joystick timeout must be positive")
        if self._trigger_pressed >= self._trigger_released:
            raise ValueError("Pressed threshold must be below released threshold")

        self._publisher = self.create_publisher(DriveCommand, "/laksa/command", 10)
        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_timer(1.0 / publish_rate, self._publish_command)

        self._last_joy_ns = 0
        self._steering = 0.0
        self._trigger = 0.0
        self._armed = False
        self._traction_active = False
        self.get_logger().info(
            "Waiting for the right trigger to be released before arming"
        )

    def _joy_callback(self, message: Joy) -> None:
        required_axes = max(self._steering_axis, self._trigger_axis) + 1
        if len(message.axes) < required_axes:
            self.get_logger().error(
                f"Joy message has {len(message.axes)} axes; need {required_axes}",
                throttle_duration_sec=2.0,
            )
            return

        self._last_joy_ns = self.get_clock().now().nanoseconds
        self._steering = float(message.axes[self._steering_axis])
        self._trigger = float(message.axes[self._trigger_axis])
        if not self._armed and self._trigger >= self._trigger_released:
            self._armed = True
            self.get_logger().info("Xbox traction armed; trigger is released")

    def _publish_command(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        joy_fresh = (
            self._last_joy_ns != 0
            and now_ns - self._last_joy_ns <= self._joy_timeout_ns
        )
        traction_active = (
            joy_fresh and self._armed and self._trigger <= self._trigger_pressed
        )

        command = DriveCommand()
        if joy_fresh and self._armed:
            command.steering_angle_rad = self._steering_scale * self._steering
        else:
            command.steering_angle_rad = 0.0
        command.speed_mps = self._pressed_speed if traction_active else 0.0
        command.brake = not joy_fresh
        self._publisher.publish(command)

        if traction_active != self._traction_active:
            self._traction_active = traction_active
            state = "ACTIVE" if traction_active else "STOPPED"
            self.get_logger().info(f"Traction {state}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XboxDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
