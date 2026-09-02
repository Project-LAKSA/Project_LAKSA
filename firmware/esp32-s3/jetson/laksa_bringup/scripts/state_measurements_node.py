#!/usr/bin/env python3

"""Expose measured VESC velocity and a conservative planar IMU input."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from laksa_interfaces.msg import VehicleState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class StateMeasurements(Node):
    """Adapt ESP32 telemetry into standard inputs for robot_localization."""

    def __init__(self) -> None:
        super().__init__("state_measurements")
        self.declare_parameter("odom_frame", "laksa_odom")
        self.declare_parameter("base_frame", "laksa_base_footprint")
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("velocity_variance", 0.02)
        self.declare_parameter("gyro_z_variance", 0.015)
        self.declare_parameter("gyro_z_sign", 1.0)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._timeout_ms = int(
            float(self.get_parameter("state_timeout_sec").value) * 1000.0
        )
        self._velocity_variance = float(
            self.get_parameter("velocity_variance").value
        )
        self._gyro_variance = float(
            self.get_parameter("gyro_z_variance").value
        )
        self._gyro_sign = float(self.get_parameter("gyro_z_sign").value)
        if self._timeout_ms <= 0:
            raise ValueError("state_timeout_sec must be positive")
        if self._velocity_variance <= 0.0 or self._gyro_variance <= 0.0:
            raise ValueError("measurement variances must be positive")
        if self._gyro_sign not in (-1.0, 1.0):
            raise ValueError("gyro_z_sign must be -1 or 1")

        self._vesc_pub = self.create_publisher(
            Odometry, "/laksa/vesc_odom", qos_profile_sensor_data
        )
        self._imu_pub = self.create_publisher(
            Imu, "/laksa/imu/ekf", qos_profile_sensor_data
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_subscription(
            VehicleState,
            "/laksa/state",
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_diagnostics)
        self._last_state_ns = 0
        self._vesc_valid = False
        self._imu_valid = False

    def _message_stamp(self, message: VehicleState):
        if int(message.stamp.sec) or int(message.stamp.nanosec):
            return message.stamp
        return self.get_clock().now().to_msg()

    def _state_callback(self, message: VehicleState) -> None:
        self._last_state_ns = self.get_clock().now().nanoseconds
        stamp = self._message_stamp(message)
        vesc = message.vesc
        speed = float(vesc.vehicle_linear_velocity_mps)
        self._vesc_valid = (
            int(vesc.telemetry_sequence) > 0
            and int(vesc.telemetry_age_ms) <= self._timeout_ms
            and math.isfinite(speed)
        )
        if self._vesc_valid:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.pose.pose.orientation.w = 1.0
            # Pose is intentionally unavailable: only twist.linear.x is fused.
            for index in (0, 7, 14, 21, 28, 35):
                odom.pose.covariance[index] = 1.0e6
                odom.twist.covariance[index] = 1.0e6
            odom.twist.twist.linear.x = speed
            odom.twist.covariance[0] = self._velocity_variance
            self._vesc_pub.publish(odom)

        gyro_z = float(message.angular_velocity_rad_s.z) * self._gyro_sign
        self._imu_valid = bool(message.imu_available) and math.isfinite(gyro_z)
        if self._imu_valid:
            imu = Imu()
            imu.header.stamp = stamp
            # Firmware axes are treated as base-aligned for planar gyro Z.
            # Absolute orientation and linear acceleration remain unavailable.
            imu.header.frame_id = self._base_frame
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.z = gyro_z
            imu.angular_velocity_covariance = [
                1.0e6, 0.0, 0.0,
                0.0, 1.0e6, 0.0,
                0.0, 0.0, self._gyro_variance,
            ]
            imu.linear_acceleration_covariance[0] = -1.0
            self._imu_pub.publish(imu)

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        age_sec = (
            math.inf
            if not self._last_state_ns
            else (now.nanoseconds - self._last_state_ns) / 1.0e9
        )
        fresh = age_sec <= self._timeout_ms / 1000.0
        status = DiagnosticStatus()
        status.name = "laksa_state_measurements"
        status.hardware_id = "esp32-vesc-bno08x"
        status.level = (
            DiagnosticStatus.OK
            if fresh and self._vesc_valid
            else DiagnosticStatus.ERROR
        )
        status.message = (
            "VESC velocity and planar IMU adapter active"
            if status.level == DiagnosticStatus.OK
            else "ESP32/VESC measurement input stale or invalid"
        )
        status.values = [
            KeyValue(key="state_age_sec", value=f"{age_sec:.3f}"),
            KeyValue(key="vesc_velocity_valid", value=str(self._vesc_valid)),
            KeyValue(key="car_gyro_z_valid", value=str(self._imu_valid)),
        ]
        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status = [status]
        self._diagnostics_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = StateMeasurements()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

