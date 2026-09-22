#!/usr/bin/env python3
"""Bounded MPPI FollowPath probe for an isolated, actuator-free ROS domain."""

import argparse
import json
import math
import struct
import threading
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from laksa_control_math import limited_ackermann_command


class FollowPathProbe(Node):
    def __init__(self):
        super().__init__("laksa_followpath_dry_run_probe")
        self._odom_pub = self.create_publisher(Odometry, "/laksa/odometry/fused", 10)
        self._scan_pub = self.create_publisher(
            LaserScan, "/laksa/lidar/scan_validated", qos_profile_sensor_data
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/laksa/nav_cmd_vel", self._cmd_cb, 10)
        self._client = ActionClient(self, FollowPath, "/follow_path")
        self._commands = []
        self.create_timer(0.05, self._publish_fixture)

    def _cmd_cb(self, message):
        self._commands.append(
            (time.monotonic(), float(message.linear.x), float(message.angular.z))
        )

    def _publish_fixture(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_footprint"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [float("inf")] * 361
        self._scan_pub.publish(scan)

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "base_footprint"
        cloud.height = 1
        cloud.width = 1
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12
        cloud.is_dense = True
        cloud.data = struct.pack("<fff", 3.0, 2.0, 0.30)
        self._cloud_pub.publish(cloud)

    def run_path(self, name, coordinates, duration=2.5):
        goal = FollowPath.Goal()
        goal.controller_id = "FollowPath"
        goal.path = Path()
        goal.path.header.stamp = self.get_clock().now().to_msg()
        goal.path.header.frame_id = "map"
        for x, y, yaw in coordinates:
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.path.poses.append(pose)
        self._commands.clear()
        sent = self._client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not sent.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not sent.done() or not sent.result().accepted:
            return {"name": name, "accepted": False, "commands": 0}
        handle = sent.result()
        time.sleep(duration)
        canceled = handle.cancel_goal_async()
        cancel_deadline = time.monotonic() + 3.0
        while not canceled.done() and time.monotonic() < cancel_deadline:
            time.sleep(0.02)
        samples = list(self._commands)
        finite = all(math.isfinite(v) and math.isfinite(w) for _, v, w in samples)
        nonzero = [(t, v, w) for t, v, w in samples if abs(v) > 1e-3 or abs(w) > 1e-3]
        rotate = sum(abs(v) <= 0.01 and abs(w) > 0.01 for _, v, w in samples)
        steering = []
        converted_speeds = []
        for _, v, w in samples:
            converted_speed, angle = limited_ackermann_command(
                v, w, 0.15, 0.324, 0.523, 0.288, 0.523
            )
            converted_speeds.append(converted_speed)
            steering.append(angle)
        span = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0.0
        return {
            "name": name,
            "accepted": True,
            "commands": len(samples),
            "nonzero_commands": len(nonzero),
            "effective_hz": round((len(samples) - 1) / span, 3) if span > 0 else 0.0,
            "finite": finite,
            "max_abs_speed": max((abs(v) for _, v, _ in samples), default=0.0),
            "max_abs_converted_speed": max(
                (abs(v) for v in converted_speeds), default=0.0
            ),
            "max_abs_yaw_rate": max((abs(w) for _, _, w in samples), default=0.0),
            "max_abs_steering": max((abs(a) for a in steering), default=0.0),
            "rotate_in_place_commands": rotate,
            "negative_speed_commands": sum(v < -1e-3 for _, v, _ in samples),
        }


def paths():
    straight = [(i * 0.1, 0.0, 0.0) for i in range(21)]
    left = [(1.2 * math.sin(t), 1.2 * (1.0 - math.cos(t)), t) for t in [i * 0.05 for i in range(25)]]
    right = [(x, -y, -yaw) for x, y, yaw in left]
    s_curve = []
    for i in range(31):
        x = i * 0.08
        y = 0.25 * math.sin(math.pi * x / 1.2)
        dy = 0.25 * math.pi / 1.2 * math.cos(math.pi * x / 1.2)
        s_curve.append((x, y, math.atan(dy)))
    reverse = [(-i * 0.08, 0.0, 0.0) for i in range(16)]
    return [("straight", straight), ("left", left), ("right", right), ("s_curve", s_curve), ("reverse_probe", reverse)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    rclpy.init()
    node = FollowPathProbe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        if not node._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowPath action is unavailable")
        results = [node.run_path(name, path) for name, path in paths()]
        required = results[:4]
        passed = all(
            item["accepted"] and item["finite"] and item["nonzero_commands"] > 0
            and item["max_abs_converted_speed"] <= 0.150001
            and item["max_abs_steering"] <= 0.523001
            and item["rotate_in_place_commands"] == 0
            for item in required
        )
        report = {"pass": passed, "scenarios": results}
        text = json.dumps(report, indent=2, sort_keys=True)
        print(text)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as stream:
                stream.write(text + "\n")
        raise SystemExit(0 if passed else 1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
