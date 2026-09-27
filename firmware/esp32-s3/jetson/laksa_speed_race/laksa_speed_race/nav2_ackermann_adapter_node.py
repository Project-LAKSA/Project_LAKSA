"""Thin deterministic TwistStamped-to-Ackermann adapter for C1.2."""

from __future__ import annotations

import csv
import math
from pathlib import Path

from .nav2_raceline_node import load_frozen_raceline, monotonic_progress_index, unroll_closed_raceline


WHEELBASE_M = 0.324
STEERING_LIMIT_RAD = 0.288
ZERO_SPEED_EPSILON_MPS = 1.0e-9


def twist_to_ackermann(
    linear_mps: float,
    angular_rps: float,
    *,
    wheelbase_m: float = WHEELBASE_M,
    steering_limit_rad: float = STEERING_LIMIT_RAD,
) -> tuple[float, float, bool]:
    """Return speed, clamped steering, saturation; no tracking intelligence."""
    if not math.isfinite(linear_mps) or not math.isfinite(angular_rps):
        raise ValueError("non-finite Nav2 command")
    if linear_mps < 0.0:
        raise ValueError("reverse command is forbidden in C1.2")
    if abs(linear_mps) <= ZERO_SPEED_EPSILON_MPS:
        if abs(angular_rps) > ZERO_SPEED_EPSILON_MPS:
            raise ValueError("C1.2 Ackermann proxy cannot rotate in place")
        return 0.0, 0.0, False
    raw = math.atan(wheelbase_m * angular_rps / linear_mps)
    applied = max(-steering_limit_rad, min(steering_limit_rad, raw))
    return linear_mps, applied, not math.isclose(raw, applied, abs_tol=1.0e-15)


def main(args: list[str] | None = None) -> None:
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PointStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class AckermannAdapter(Node):
        def __init__(self) -> None:
            super().__init__("nav2_ackermann_adapter")
            share = Path(get_package_share_directory("laksa_speed_race"))
            self.declare_parameter("output_dir", "/tmp/laksa-c1-results/nav2_trial_1")
            self.output_dir = Path(self.get_parameter("output_dir").value)
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.telemetry_stream = (self.output_dir / "controller_telemetry.csv").open("w", newline="")
            self.telemetry = csv.DictWriter(
                self.telemetry_stream,
                fieldnames=[
                    "state_stamp_ns", "sequence", "x_m", "y_m", "yaw_rad", "actual_velocity_mps",
                    "progress_index", "path_copy", "carrot_x_m", "carrot_y_m", "lookahead_distance_m",
                    "curvature_1pm", "upstream_linear_mps", "upstream_angular_rps", "delta_raw_rad",
                    "delta_applied_rad", "steering_saturated",
                ],
            )
            self.telemetry.writeheader()
            raceline = share / "course" / "canonical" / "speed_course" / "pure_pursuit_raceline.csv"
            self.points = unroll_closed_raceline(load_frozen_raceline(raceline))
            self.progress_index = 0
            self.sequence = 0
            self.odom_by_stamp: dict[int, tuple[float, float, float, float]] = {}
            self.carrot_by_stamp: dict[int, tuple[float, float]] = {}
            self.publisher = self.create_publisher(AckermannDriveStamped, "/c1/drive_request", 10)
            self.create_subscription(Odometry, "/c1/odom", self.on_odom, 10)
            self.create_subscription(PointStamped, "/c1/lookahead_point", self.on_carrot, 10)
            self.create_subscription(TwistStamped, "/c1/nav2_cmd_vel", self.on_twist, 10)

        @staticmethod
        def stamp_ns(stamp) -> int:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        def on_odom(self, message: Odometry) -> None:
            yaw = 2.0 * math.atan2(message.pose.pose.orientation.z, message.pose.pose.orientation.w)
            stamp = self.stamp_ns(message.header.stamp)
            self.odom_by_stamp[stamp] = (
                float(message.pose.pose.position.x), float(message.pose.pose.position.y), yaw,
                float(message.twist.twist.linear.x),
            )
            if len(self.odom_by_stamp) > 4:
                del self.odom_by_stamp[min(self.odom_by_stamp)]

        def on_carrot(self, message: PointStamped) -> None:
            self.carrot_by_stamp[self.stamp_ns(message.header.stamp)] = (
                float(message.point.x), float(message.point.y)
            )
            if len(self.carrot_by_stamp) > 4:
                del self.carrot_by_stamp[min(self.carrot_by_stamp)]

        def on_twist(self, message: TwistStamped) -> None:
            linear = float(message.twist.linear.x)
            angular = float(message.twist.angular.z)
            speed, steering, saturated = twist_to_ackermann(linear, angular)
            raw = 0.0 if speed == 0.0 else math.atan(WHEELBASE_M * angular / speed)
            request = AckermannDriveStamped()
            request.header = message.header
            request.header.frame_id = "c1/base_link"
            request.drive.speed = speed
            request.drive.steering_angle = steering
            self.publisher.publish(request)

            stamp = self.stamp_ns(message.header.stamp)
            state = self.odom_by_stamp.get(stamp)
            carrot = self.carrot_by_stamp.get(stamp)
            if state is not None:
                self.progress_index = monotonic_progress_index(
                    self.points, state[0], state[1], self.progress_index
                )
            self.sequence += 1
            self.telemetry.writerow(
                {
                    "state_stamp_ns": stamp,
                    "sequence": self.sequence,
                    "x_m": "" if state is None else state[0],
                    "y_m": "" if state is None else state[1],
                    "yaw_rad": "" if state is None else state[2],
                    "actual_velocity_mps": "" if state is None else state[3],
                    "progress_index": self.progress_index,
                    "path_copy": self.progress_index // 546,
                    "carrot_x_m": "" if carrot is None else carrot[0],
                    "carrot_y_m": "" if carrot is None else carrot[1],
                    "lookahead_distance_m": "" if carrot is None else math.hypot(*carrot),
                    "curvature_1pm": 0.0 if speed == 0.0 else angular / speed,
                    "upstream_linear_mps": linear,
                    "upstream_angular_rps": angular,
                    "delta_raw_rad": raw,
                    "delta_applied_rad": steering,
                    "steering_saturated": int(saturated),
                }
            )
            self.telemetry_stream.flush()

        def destroy_node(self):
            self.telemetry_stream.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = AckermannAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
