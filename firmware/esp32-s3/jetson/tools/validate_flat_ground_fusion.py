#!/usr/bin/env python3
"""Bounded, read-only validation of LAKSA's flat-ground fused odometry."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_msgs.msg import TFMessage


def _rpy(quaternion) -> tuple[float, float, float]:
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _angle_delta(end: float, start: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


class FusionProbe(Node):
    def __init__(self) -> None:
        super().__init__("laksa_flat_ground_fusion_probe")
        self.samples = {"raw_zed": [], "fused_base": []}
        self.tf_edges = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Odometry,
            "/zed/zed_node/odom",
            lambda message: self._record("raw_zed", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/laksa/odometry/fused",
            lambda message: self._record("fused_base", message),
            qos_profile_sensor_data,
        )
        dynamic_tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        static_tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage,
            "/tf",
            lambda message, info: self._record_tf("/tf", message, info.publisher_gid),
            dynamic_tf_qos,
        )
        self.create_subscription(
            TFMessage,
            "/tf_static",
            lambda message, info: self._record_tf("/tf_static", message, info.publisher_gid),
            static_tf_qos,
        )

    def _record(self, name: str, message: Odometry) -> None:
        roll, pitch, yaw = _rpy(message.pose.pose.orientation)
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            roll,
            pitch,
            yaw,
        )
        if all(math.isfinite(value) for value in values):
            self.samples[name].append(values)

    def _record_tf(self, topic: str, message: TFMessage, publisher_gid) -> None:
        gid = tuple(int(value) for value in publisher_gid)
        for transform in message.transforms:
            edge = f"{transform.header.frame_id}->{transform.child_frame_id}"
            self.tf_edges[edge] = {"topic": topic, "publisher_gid": gid}

    def _topic_publishers(self, topic: str) -> list[str]:
        return sorted(
            {
                f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
                for info in self.get_publishers_info_by_topic(topic)
            }
        )

    def _tf_authorities(self) -> dict:
        endpoints = {}
        for topic in ("/tf", "/tf_static"):
            for info in self.get_publishers_info_by_topic(topic):
                gid = tuple(int(value) for value in info.endpoint_gid)
                endpoints[(topic, gid)] = (
                    f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
                )
        required = ("map->odom", "odom->base_footprint", "base_footprint->zed_camera_link")
        return {
            edge: {
                "topic": self.tf_edges[edge]["topic"],
                "publisher": endpoints.get(
                    (self.tf_edges[edge]["topic"], self.tf_edges[edge]["publisher_gid"]),
                    "UNKNOWN",
                ),
            }
            if edge in self.tf_edges
            else {"topic": "MISSING", "publisher": "MISSING"}
            for edge in required
        }

    def result(self) -> dict:
        result = {
            "tf_publishers": self._topic_publishers("/tf"),
            "tf_static_publishers": self._topic_publishers("/tf_static"),
            "required_tf_authorities": self._tf_authorities(),
            "topics": {},
            "transforms": {},
        }
        for name, samples in self.samples.items():
            if not samples:
                result["topics"][name] = {"samples": 0}
                continue
            first, last = samples[0], samples[-1]
            result["topics"][name] = {
                "samples": len(samples),
                "xy_start_m": [first[0], first[1]],
                "xy_end_m": [last[0], last[1]],
                "yaw_delta_rad": _angle_delta(last[4], first[4]),
                "max_abs_roll_rad": max(abs(sample[2]) for sample in samples),
                "max_abs_pitch_rad": max(abs(sample[3]) for sample in samples),
            }
        for parent, child in (
            ("map", "odom"),
            ("odom", "base_footprint"),
            ("base_footprint", "zed_camera_link"),
        ):
            edge = f"{parent}->{child}"
            try:
                transform = self._tf_buffer.lookup_transform(parent, child, Time(), Duration(seconds=1.0))
                roll, pitch, yaw = _rpy(transform.transform.rotation)
                result["transforms"][edge] = {
                    "present": True,
                    "rpy_rad": [roll, pitch, yaw],
                }
            except TransformException as error:
                result["transforms"][edge] = {"present": False, "error": str(error)}
        fused = result["topics"]["fused_base"]
        result["flat_ground_base_bounded"] = bool(
            fused.get("samples", 0)
            and fused["max_abs_roll_rad"] <= math.radians(1.0)
            and fused["max_abs_pitch_rad"] <= math.radians(1.0)
        )
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 600.0:
        raise SystemExit("duration must be between 1 and 600 seconds")
    rclpy.init()
    node = FusionProbe()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.result(), indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
