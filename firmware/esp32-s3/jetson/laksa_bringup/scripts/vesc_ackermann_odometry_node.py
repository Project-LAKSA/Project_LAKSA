#!/usr/bin/env python3

"""Integrate measured VESC eRPM with Ackermann steering into wheel odometry."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from laksa_interfaces.msg import VehicleState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty
from tf2_ros import TransformBroadcaster


class VescAckermannOdometry(Node):
    def __init__(self) -> None:
        super().__init__("vesc_ackermann_odometry")
        parameters = {
            "odom_frame": "laksa_odom",
            "base_frame": "laksa_base_footprint",
            "motor_pole_pairs": 2.0,
            "gear_reduction": 11.82,
            "wheel_diameter_m": 0.109,
            "wheelbase_m": 0.324,
            "servo_reported_limit_rad": 0.523,
            "left_road_wheel_limit_rad": 0.523,
            "right_road_wheel_limit_rad": 0.288,
            "forward_sign": 1.0,
            "state_timeout_sec": 1.0,
            "publish_rate_hz": 30.0,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._pole_pairs = float(self.get_parameter("motor_pole_pairs").value)
        self._gear_reduction = float(self.get_parameter("gear_reduction").value)
        self._wheel_diameter = float(
            self.get_parameter("wheel_diameter_m").value
        )
        self._wheelbase = float(self.get_parameter("wheelbase_m").value)
        self._servo_limit = float(
            self.get_parameter("servo_reported_limit_rad").value
        )
        self._left_wheel_limit = float(
            self.get_parameter("left_road_wheel_limit_rad").value
        )
        self._right_wheel_limit = float(
            self.get_parameter("right_road_wheel_limit_rad").value
        )
        self._forward_sign = float(self.get_parameter("forward_sign").value)
        self._state_timeout_ns = int(
            float(self.get_parameter("state_timeout_sec").value) * 1e9
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if min(
            self._pole_pairs,
            self._gear_reduction,
            self._wheel_diameter,
            self._wheelbase,
            self._servo_limit,
            self._left_wheel_limit,
            self._right_wheel_limit,
            publish_rate,
        ) <= 0.0:
            raise ValueError("Odometry geometry and rates must be positive")

        self._publisher = self.create_publisher(Odometry, "/laksa/odom", 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(
            VehicleState,
            "/laksa/state",
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Empty, "/laksa/reset_odometry", self._reset_callback, 10
        )
        self.create_timer(1.0 / publish_rate, self._publish)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._last_state_ns = 0
        self._last_integrated_ns = 0
        self.get_logger().info(
            f"VESC Ackermann odometry: ratio={self._gear_reduction:.3f}, "
            f"wheel={self._wheel_diameter:.3f} m, pole_pairs={self._pole_pairs:.0f}"
        )

    def _reset_callback(self, _message: Empty) -> None:
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._last_integrated_ns = self.get_clock().now().nanoseconds
        self.get_logger().warn("Wheel odometry reset requested")

    def _erpm_to_mps(self, erpm: float) -> float:
        return (
            erpm
            * math.pi
            * self._wheel_diameter
            * self._forward_sign
            / (60.0 * self._gear_reduction * self._pole_pairs)
        )

    def _state_callback(self, message: VehicleState) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_integrated_ns:
            dt = (now_ns - self._last_integrated_ns) / 1e9
            if 0.0 < dt <= 0.5:
                self._x += self._speed * math.cos(self._yaw) * dt
                self._y += self._speed * math.sin(self._yaw) * dt
                self._yaw = math.atan2(
                    math.sin(self._yaw + self._yaw_rate * dt),
                    math.cos(self._yaw + self._yaw_rate * dt),
                )
        self._last_integrated_ns = now_ns
        self._last_state_ns = now_ns

        telemetry_fresh = (
            int(message.vesc.telemetry_sequence) > 0
            and int(message.vesc.telemetry_age_ms)
            <= self._state_timeout_ns // 1_000_000
        )
        if telemetry_fresh:
            self._speed = self._erpm_to_mps(float(message.vesc.measured_erpm))
            servo_steering = float(message.steering_current_rad)
            physical_limit = (
                self._left_wheel_limit
                if servo_steering >= 0.0
                else self._right_wheel_limit
            )
            # The ESP32 reports normalized servo position, not measured road-
            # wheel angle. Account for the asymmetric steering calibration so
            # wheel odometry does not invent turns the chassis cannot make.
            steering = servo_steering * physical_limit / self._servo_limit
            self._yaw_rate = (
                self._speed * math.tan(steering) / self._wheelbase
                if abs(self._speed) > 1e-4
                else 0.0
            )
        else:
            self._speed = 0.0
            self._yaw_rate = 0.0

    @staticmethod
    def _yaw_quaternion(yaw: float):
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _publish(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        speed = self._speed
        yaw_rate = self._yaw_rate
        if (
            self._last_state_ns == 0
            or now_ns - self._last_state_ns > self._state_timeout_ns
        ):
            speed = 0.0
            yaw_rate = 0.0

        qx, qy, qz, qw = self._yaw_quaternion(self._yaw)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05
        self._publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VescAckermannOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
