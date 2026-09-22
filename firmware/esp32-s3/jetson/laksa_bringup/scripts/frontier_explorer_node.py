#!/usr/bin/env python3

"""Small frontier selector that feeds Nav2 only while autonomous mode is enabled."""

import math
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        defaults = {
            "map_topic": "/map",
            "map_frame": "map",
            "base_frame": "laksa_base_footprint",
            "minimum_cluster_cells": 8,
            "minimum_goal_distance_m": 0.60,
            "goal_standoff_m": 0.25,
            "blacklist_radius_m": 0.50,
            "goal_timeout_sec": 45.0,
            "completion_stability_sec": 10.0,
            "maximum_consecutive_failures": 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._minimum_cluster = int(
            self.get_parameter("minimum_cluster_cells").value
        )
        self._minimum_distance = float(
            self.get_parameter("minimum_goal_distance_m").value
        )
        self._standoff = float(self.get_parameter("goal_standoff_m").value)
        self._blacklist_radius = float(
            self.get_parameter("blacklist_radius_m").value
        )
        self._goal_timeout_ns = int(
            float(self.get_parameter("goal_timeout_sec").value) * 1e9
        )
        self._completion_stability_ns = int(
            float(self.get_parameter("completion_stability_sec").value) * 1e9
        )
        self._maximum_failures = int(
            self.get_parameter("maximum_consecutive_failures").value
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            "/laksa/exploration_enabled",
            self._mode_callback,
            latched_qos,
        )
        self.create_subscription(
            OccupancyGrid, self._map_topic, self._map_callback, latched_qos
        )
        self._complete_pub = self.create_publisher(
            Bool, "/laksa/exploration_complete", latched_qos
        )
        self._status_pub = self.create_publisher(
            String, "/laksa/exploration_status", latched_qos
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._navigate = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0, self._update)

        self._enabled = False
        self._map = None
        self._goal_handle = None
        self._goal_started_ns = 0
        self._goal_pending = False
        self._goal_xy = None
        self._blacklist = []
        self._no_frontier_since_ns = 0
        self._complete = False
        self._consecutive_failures = 0
        self._publish_complete(False)
        self._publish_status("IDLE")
        self.get_logger().info("Frontier exploration idle; waiting for A-hold mode")

    def _publish_complete(self, complete: bool) -> None:
        self._complete = complete
        message = Bool()
        message.data = complete
        self._complete_pub.publish(message)

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_pub.publish(message)

    def _mode_callback(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        if enabled:
            self._blacklist.clear()
            self._consecutive_failures = 0
            self._no_frontier_since_ns = 0
            self._publish_complete(False)
            self._publish_status("SEARCHING")
            self.get_logger().warn("Frontier exploration ENABLED")
        else:
            self.get_logger().warn("Frontier exploration disabled; canceling goal")
            self._cancel_goal()
            self._publish_status("COMPLETE" if self._complete else "IDLE")

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._map = message

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._goal_pending = False
        self._goal_started_ns = 0
        self._goal_xy = None

    def _robot_xy(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for map->base transform: {error}",
                throttle_duration_sec=5.0,
            )
            return None
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    @staticmethod
    def _neighbors(index, width, height, diagonals=False):
        x = index % width
        y = index // width
        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if diagonals:
            offsets += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield ny * width + nx

    def _frontier_clusters(self):
        grid = self._map
        width = int(grid.info.width)
        height = int(grid.info.height)
        data = grid.data
        frontier = set()
        for index, value in enumerate(data):
            if value != 0:
                continue
            if any(
                data[neighbor] < 0
                for neighbor in self._neighbors(index, width, height)
            ):
                frontier.add(index)

        clusters = []
        while frontier:
            seed = frontier.pop()
            queue = deque([seed])
            cluster = [seed]
            while queue:
                current = queue.popleft()
                for neighbor in self._neighbors(
                    current, width, height, diagonals=True
                ):
                    if neighbor in frontier:
                        frontier.remove(neighbor)
                        queue.append(neighbor)
                        cluster.append(neighbor)
            if len(cluster) >= self._minimum_cluster:
                clusters.append(cluster)
        return clusters

    def _cell_xy(self, index):
        info = self._map.info
        x_cell = index % int(info.width)
        y_cell = index // int(info.width)
        return (
            float(info.origin.position.x) + (x_cell + 0.5) * info.resolution,
            float(info.origin.position.y) + (y_cell + 0.5) * info.resolution,
        )

    def _blacklisted(self, x, y):
        return any(
            math.hypot(x - bx, y - by) < self._blacklist_radius
            for bx, by in self._blacklist
        )

    def _known_free_with_clearance(self, x, y, clearance_m=0.25):
        info = self._map.info
        width, height = int(info.width), int(info.height)
        column = math.floor((x - float(info.origin.position.x)) / info.resolution)
        row = math.floor((y - float(info.origin.position.y)) / info.resolution)
        radius = max(1, math.ceil(clearance_m / info.resolution))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                check_x, check_y = column + dx, row + dy
                if not (0 <= check_x < width and 0 <= check_y < height):
                    return False
                if self._map.data[check_y * width + check_x] != 0:
                    return False
        return True

    def _select_goal(self, robot_x, robot_y):
        candidates = []
        clusters = self._frontier_clusters()
        for cluster in clusters:
            points = [self._cell_xy(index) for index in cluster]
            x = sum(point[0] for point in points) / len(points)
            y = sum(point[1] for point in points) / len(points)
            distance = math.hypot(x - robot_x, y - robot_y)
            if distance < self._minimum_distance or self._blacklisted(x, y):
                continue
            dx = x - robot_x
            dy = y - robot_y
            # Walk back from the unknown boundary until the whole vehicle has
            # known-free clearance. A frontier centroid by itself is commonly
            # too close to an inflated wall for an Ackermann footprint.
            retreat = self._standoff
            while distance - retreat >= self._minimum_distance:
                scale = (distance - retreat) / distance
                goal_x = robot_x + dx * scale
                goal_y = robot_y + dy * scale
                if self._known_free_with_clearance(goal_x, goal_y):
                    score = (
                        math.hypot(goal_x - robot_x, goal_y - robot_y)
                        - min(len(cluster), 100)
                        * self._map.info.resolution
                        * 0.10
                    )
                    candidates.append((score, goal_x, goal_y))
                    break
                retreat += max(0.05, float(self._map.info.resolution))
        if not candidates:
            return None, len(clusters)

        _, goal_x, goal_y = min(candidates)
        return (goal_x, goal_y), len(clusters)

    def _record_failure(self, reason: str) -> None:
        self._consecutive_failures += 1
        self.get_logger().warning(
            f"{reason}; consecutive failures="
            f"{self._consecutive_failures}/{self._maximum_failures}"
        )
        if self._consecutive_failures >= self._maximum_failures:
            self._enabled = False
            self._cancel_goal()
            self._publish_status("BLOCKED")
            self.get_logger().error(
                "Exploration BLOCKED after repeated Nav2 failures; "
                "operator inspection is required"
            )

    def _send_goal(self, x, y, robot_x, robot_y) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = math.atan2(y - robot_y, x - robot_x)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pending = True
        self._publish_status("NAVIGATING_TO_FRONTIER")
        self._goal_xy = (x, y)
        future = self._navigate.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.get_logger().info(f"Frontier goal: x={x:.2f}, y={y:.2f}")

    def _goal_response(self, future) -> None:
        self._goal_pending = False
        try:
            handle = future.result()
        except Exception as error:  # rclpy action transport failure
            self.get_logger().error(f"Goal request failed: {error}")
            return
        if not handle.accepted:
            if self._goal_xy is not None:
                self._blacklist.append(self._goal_xy)
            self._goal_xy = None
            self._record_failure("Nav2 rejected frontier goal")
            if self._enabled:
                self._publish_status("SEARCHING")
            return
        self._goal_handle = handle
        self._goal_started_ns = self.get_clock().now().nanoseconds
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            if self._goal_xy is not None:
                self._blacklist.append(self._goal_xy)
            self._record_failure(f"Frontier goal ended with status {status}")
        else:
            self._consecutive_failures = 0
        self._goal_handle = None
        self._goal_started_ns = 0
        self._goal_xy = None
        if self._enabled:
            self._publish_status("SEARCHING")

    def _update(self) -> None:
        if not self._enabled or self._map is None or self._goal_pending:
            return
        if self._goal_handle is not None:
            if (
                self._goal_started_ns
                and self.get_clock().now().nanoseconds - self._goal_started_ns
                > self._goal_timeout_ns
            ):
                self.get_logger().warning("Frontier goal timeout; canceling")
                if self._goal_xy is not None:
                    self._blacklist.append(self._goal_xy)
                self._cancel_goal()
                self._publish_status("SEARCHING")
            return
        if not self._navigate.wait_for_server(timeout_sec=0.05):
            self.get_logger().warning(
                "Waiting for Nav2 NavigateToPose action",
                throttle_duration_sec=5.0,
            )
            return
        robot = self._robot_xy()
        if robot is None:
            return
        selected, frontier_count = self._select_goal(*robot)
        if selected is None:
            if frontier_count > 0:
                self._no_frontier_since_ns = 0
                self._enabled = False
                self._publish_status("BLOCKED")
                self.get_logger().warning(
                    f"Exploration BLOCKED: {frontier_count} frontier clusters "
                    "remain but none is currently eligible",
                    throttle_duration_sec=5.0,
                )
                return
            now_ns = self.get_clock().now().nanoseconds
            if self._no_frontier_since_ns == 0:
                self._no_frontier_since_ns = now_ns
                self._publish_status("VERIFYING_COMPLETE")
            elif now_ns - self._no_frontier_since_ns >= self._completion_stability_ns:
                self._publish_complete(True)
                self._publish_status("COMPLETE")
                self._enabled = False
                self.get_logger().warn(
                    "Exploration COMPLETE: no eligible frontier remained for "
                    f"{self._completion_stability_ns / 1e9:.0f} seconds"
                )
            self.get_logger().info(
                "No reachable frontier candidate yet",
                throttle_duration_sec=5.0,
            )
            return
        self._no_frontier_since_ns = 0
        self._send_goal(*selected, *robot)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
