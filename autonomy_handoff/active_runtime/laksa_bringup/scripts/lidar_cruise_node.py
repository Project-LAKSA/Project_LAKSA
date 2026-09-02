#!/usr/bin/env python3

"""Run the simulation-tested forward-priority Ackermann rollout controller."""

import math

import rclpy
from geometry_msgs.msg import Twist
from lidar_cruise_math import percentile, wrap_angle, yaw_rate_from_steering
from lidar_rollout_core import (
    AckermannRolloutController,
    Pose2D,
    RolloutParameters,
    VehicleGeometry,
    VisitMemory,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class BaseFrameScan:
    """Minimal scan interface consumed by the ROS-independent controller."""

    def __init__(self, ranges, angles, points_base, maximum: float) -> None:
        self.ranges = ranges
        self.angles = angles
        self.points_base = points_base
        self.maximum = maximum

    def sector(self, center: float, half_width: float, fraction: float) -> float:
        values = [
            distance
            for distance, angle in zip(self.ranges, self.angles)
            if abs(wrap_angle(angle - center)) <= half_width
        ]
        return percentile(values, fraction, self.maximum)


class LidarCruise(Node):
    def __init__(self) -> None:
        super().__init__("lidar_cruise")
        defaults = {
            "forward_speed_mps": 0.2414,
            "reverse_speed_mps": 0.2173,
            "wheelbase_m": 0.324,
            "lidar_x_m": 0.315,
            "lidar_yaw_rad": math.pi,
            "left_steering_limit_rad": 0.523,
            "right_steering_limit_rad": 0.288,
            "horizon_sec": 3.4,
            "clearance_weight": 3.5,
            "novelty_weight": 1.5,
            "continuity_weight": 0.30,
            "steering_weight": 0.20,
            "minimum_clearance_m": 0.20,
            "blocked_before_reverse_sec": 2.0,
            "reverse_duration_sec": 1.80,
            "forward_recovery_sec": 2.50,
            "reverse_distance_m": 0.36,
            "forward_recovery_yaw_rad": 0.75,
            "recovery_phase_timeout_sec": 5.0,
            "reverse_cooldown_sec": 10.0,
            "scan_timeout_sec": 0.50,
            "odom_timeout_sec": 0.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {
            name: float(self.get_parameter(name).value) for name in defaults
        }
        self._lidar_x = values["lidar_x_m"]
        self._lidar_yaw = values["lidar_yaw_rad"]
        self._scan_timeout_ns = int(values["scan_timeout_sec"] * 1e9)
        self._odom_timeout_ns = int(values["odom_timeout_sec"] * 1e9)
        geometry = VehicleGeometry(
            wheelbase=values["wheelbase_m"],
            max_left=values["left_steering_limit_rad"],
            max_right=values["right_steering_limit_rad"],
        )
        params = RolloutParameters(
            horizon_sec=values["horizon_sec"],
            clearance_weight=values["clearance_weight"],
            novelty_weight=values["novelty_weight"],
            continuity_weight=values["continuity_weight"],
            steering_weight=values["steering_weight"],
            minimum_clearance=values["minimum_clearance_m"],
            blocked_sec=values["blocked_before_reverse_sec"],
            reverse_sec=values["reverse_duration_sec"],
            forward_recovery_sec=values["forward_recovery_sec"],
            reverse_distance_m=values["reverse_distance_m"],
            forward_recovery_yaw_rad=values["forward_recovery_yaw_rad"],
            recovery_phase_timeout_sec=values["recovery_phase_timeout_sec"],
            cooldown_sec=values["reverse_cooldown_sec"],
            forward_speed=values["forward_speed_mps"],
            reverse_speed=values["reverse_speed_mps"],
        )
        self._geometry = geometry
        self._controller = AckermannRolloutController(params, geometry)
        self._memory = VisitMemory()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._command_pub = self.create_publisher(
            Twist, "/laksa/lidar_cruise_cmd_vel", 10
        )
        self._status_pub = self.create_publisher(
            String, "/laksa/exploration_status", latched
        )
        self.create_subscription(
            Bool, "/laksa/exploration_enabled", self._enabled_callback, latched
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/laksa/odom", self._odom_callback, 10)
        self.create_timer(0.10, self._watchdog)

        self._enabled = False
        self._pose = None
        self._last_scan_ns = 0
        self._last_odom_ns = 0
        self._last_status = ""
        self._publish_status("IDLE")

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _publish_status(self, value: str) -> None:
        if value == self._last_status:
            return
        message = String()
        message.data = value
        self._status_pub.publish(message)
        self._last_status = value
        self.get_logger().info(f"LiDAR rollout state -> {value}")

    def _publish_stop(self, status: str) -> None:
        self._command_pub.publish(Twist())
        self._publish_status(status)

    def _enabled_callback(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        self._controller.reset()
        self._memory.reset()
        if enabled:
            self._publish_status("WAITING_FOR_LOCALIZATION_AND_SCAN")
        else:
            self._publish_stop("IDLE")

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        values = (position.x, position.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(float(value)) for value in values):
            self._last_odom_ns = 0
            return
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose = Pose2D(float(position.x), float(position.y), yaw)
        self._last_odom_ns = self._now_ns()

    def _base_scan(self, scan: LaserScan) -> BaseFrameScan | None:
        if not scan.ranges or not math.isfinite(float(scan.angle_increment)):
            return None
        maximum = float(scan.range_max)
        if not math.isfinite(maximum) or maximum <= 0.0:
            maximum = 12.0
        minimum = max(0.05, float(scan.range_min))
        ranges, angles, points = [], [], []
        # Match the 48-beam simulation input and bound Python controller cost
        # independently of the RPLIDAR scan mode's native sample count.
        stride = max(1, len(scan.ranges) // 48)
        for index in range(0, len(scan.ranges), stride):
            raw = scan.ranges[index]
            distance = float(raw)
            if not math.isfinite(distance) or distance < minimum:
                continue
            distance = min(distance, maximum)
            raw_angle = float(scan.angle_min) + index * float(scan.angle_increment)
            base_angle = wrap_angle(raw_angle + self._lidar_yaw)
            ranges.append(distance)
            angles.append(base_angle)
            if distance < maximum - 0.05:
                points.append(
                    (
                        self._lidar_x + math.cos(base_angle) * distance,
                        math.sin(base_angle) * distance,
                    )
                )
        return BaseFrameScan(ranges, angles, points, maximum)

    def _scan_callback(self, message: LaserScan) -> None:
        now_ns = self._now_ns()
        self._last_scan_ns = now_ns
        if not self._enabled:
            return
        if (
            self._pose is None
            or self._last_odom_ns == 0
            or now_ns - self._last_odom_ns > self._odom_timeout_ns
        ):
            self._publish_stop("ODOMETRY_TIMEOUT")
            return
        scan = self._base_scan(message)
        if scan is None:
            self._publish_stop("INVALID_SCAN")
            return
        self._memory.mark(self._pose)
        command = self._controller.command(
            self._pose, scan, self._memory, now_ns / 1e9
        )
        output = Twist()
        output.linear.x = command.speed
        output.angular.z = yaw_rate_from_steering(
            command.speed, command.steering, self._geometry.wheelbase
        )
        self._command_pub.publish(output)
        self._publish_status(command.state)

    def _watchdog(self) -> None:
        if not self._enabled:
            return
        now_ns = self._now_ns()
        if self._last_scan_ns == 0 or now_ns - self._last_scan_ns > self._scan_timeout_ns:
            self._publish_stop("SCAN_TIMEOUT")
        elif self._last_odom_ns == 0 or now_ns - self._last_odom_ns > self._odom_timeout_ns:
            self._publish_stop("ODOMETRY_TIMEOUT")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarCruise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
