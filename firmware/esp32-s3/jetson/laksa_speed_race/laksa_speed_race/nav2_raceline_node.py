"""Mechanical C1.2 adapter from the frozen cyclic CSV to a finite Nav2 Path."""

from __future__ import annotations

import csv
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path


FROZEN_RACELINE_SHA256 = "22ad91de3edbcbdf765f2cf223db53d43be820d829409da356a9def5a4c41783"
FROZEN_FULL_RACELINE_SHA256 = "1086933179481201163c4ce30d566aca6a4b6377bd723632583f3eb935e043c4"
CANONICAL_ROWS = 547
UNIQUE_ROWS = 546
UNROLLED_LAPS = 4
UNROLLED_POSES = UNIQUE_ROWS * UNROLLED_LAPS + 1


@dataclass(frozen=True)
class RacelinePoint:
    x_m: float
    y_m: float
    speed_mps: float


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_frozen_raceline(path: Path) -> list[RacelinePoint]:
    """Read the headerless C1.1 CSV and validate its duplicate closing row."""
    if sha256(path) != FROZEN_RACELINE_SHA256:
        raise ValueError("C1.1 raceline hash changed")
    with path.open(newline="") as stream:
        rows = [row for row in csv.reader(stream) if row]
    if len(rows) != CANONICAL_ROWS or any(len(row) != 3 for row in rows):
        raise ValueError(f"expected {CANONICAL_ROWS} headerless x,y,speed rows")
    points = [RacelinePoint(*(float(value) for value in row)) for row in rows]
    if not all(math.isfinite(value) for point in points for value in (point.x_m, point.y_m, point.speed_mps)):
        raise ValueError("raceline contains non-finite values")
    if points[0] != points[-1]:
        raise ValueError("C1.1 raceline must contain exactly one duplicate closing row")
    if points[0] in points[1:-1]:
        raise ValueError("C1.1 raceline closes before its terminal row")
    return points


def unroll_closed_raceline(points: list[RacelinePoint], laps: int = UNROLLED_LAPS) -> list[RacelinePoint]:
    if len(points) != CANONICAL_ROWS or points[0] != points[-1]:
        raise ValueError("expected the canonical closed 547-row C1.1 raceline")
    if laps != UNROLLED_LAPS:
        raise ValueError("C1.2 path is frozen at four copies plus one closing pose")
    unique = points[:-1]
    result = unique * laps + [unique[0]]
    if len(result) != UNROLLED_POSES:
        raise AssertionError("unexpected unrolled path length")
    return result


def monotonic_progress_index(
    points: list[RacelinePoint],
    x_m: float,
    y_m: float,
    previous_index: int,
    search_count: int = 32,
) -> int:
    """Telemetry-only bounded forward nearest-index tracker; never controls RPP."""
    if not points:
        raise ValueError("empty path")
    start = max(0, previous_index)
    stop = min(len(points), start + max(1, search_count))
    return min(
        range(start, stop),
        key=lambda index: (points[index].x_m - x_m) ** 2 + (points[index].y_m - y_m) ** 2,
    )


def main(args: list[str] | None = None) -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    class RacelineNode(Node):
        def __init__(self) -> None:
            super().__init__("nav2_raceline")
            share = Path(get_package_share_directory("laksa_speed_race"))
            self.declare_parameter(
                "raceline_path",
                str(share / "course" / "canonical" / "speed_course" / "pure_pursuit_raceline.csv"),
            )
            points = unroll_closed_raceline(load_frozen_raceline(Path(self.get_parameter("raceline_path").value)))
            message = NavPath()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            for index, point in enumerate(points):
                following = points[min(index + 1, len(points) - 1)]
                if following == point and index:
                    preceding = points[index - 1]
                    yaw = math.atan2(point.y_m - preceding.y_m, point.x_m - preceding.x_m)
                else:
                    yaw = math.atan2(following.y_m - point.y_m, following.x_m - point.x_m)
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = point.x_m
                pose.pose.position.y = point.y_m
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                message.poses.append(pose)
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.publisher = self.create_publisher(NavPath, "/c1/nav2_path", qos)
            self.publisher.publish(message)
            self.get_logger().info(
                f"Published frozen C1.2 Nav2 path: {len(message.poses)} poses, "
                f"raceline_sha256={FROZEN_RACELINE_SHA256}"
            )

    rclpy.init(args=args)
    node = RacelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
