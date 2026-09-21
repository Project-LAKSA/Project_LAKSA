#!/usr/bin/env python3
"""Persistent, read-only tf2 observer for LAKSA runtime qualification.

It intentionally has no publishers, services, action clients, or actuator
interfaces.  It replaces short-lived CLI lookups with one continuously spun
tf2 Buffer and records both numerical transforms and observed /tf traffic.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def _rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return {"roll": roll, "pitch": pitch, "yaw": math.atan2(siny, cosy)}


class TfRuntimeObserver(Node):
    def __init__(self):
        super().__init__("laksa_tf_runtime_observer")
        self.buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.samples = defaultdict(list)
        self.static = set()
        self.create_subscription(TFMessage, "/tf", self._dynamic, 100)
        self.create_subscription(TFMessage, "/tf_static", self._static, 100)

    def _record(self, message, static):
        now = time.time()
        for transform in message.transforms:
            key = f"{transform.header.frame_id}->{transform.child_frame_id}"
            self.samples[key].append({"received_unix": now, "stamp": transform.header.stamp.sec + transform.header.stamp.nanosec / 1.0e9})
            if static:
                self.static.add(key)

    def _dynamic(self, message):
        self._record(message, False)

    def _static(self, message):
        self._record(message, True)

    def observe(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        now = time.time()
        traffic = {}
        for key, values in sorted(self.samples.items()):
            received = [item["received_unix"] for item in values]
            observed_rate = 0.0
            if len(received) >= 2 and received[-1] > received[0]:
                observed_rate = (len(received) - 1) / (received[-1] - received[0])
            latest = values[-1]
            traffic[key] = {
                "static": key in self.static,
                "message_count": len(values),
                "observed_rate_hz": observed_rate,
                "latest_stamp_unix": latest["stamp"],
                "latest_received_unix": latest["received_unix"],
                "freshness_sec_at_end": now - latest["stamp"] if latest["stamp"] else None,
            }
        return traffic

    def lookup(self, parent, child):
        try:
            transform = self.buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=1.0))
        except Exception as error:
            return {"parent": parent, "child": child, "available": False, "error": f"{type(error).__name__}: {error}"}
        value = transform.transform
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1.0e9
        key = f"{parent}->{child}"
        return {
            "parent": parent, "child": child, "available": True,
            "translation_xyz_m": {"x": value.translation.x, "y": value.translation.y, "z": value.translation.z},
            "quaternion_xyzw": {"x": value.rotation.x, "y": value.rotation.y, "z": value.rotation.z, "w": value.rotation.w},
            "rpy_rad": _rpy(value.rotation),
            "stamp_unix": stamp,
            "static": key in self.static,
            "observed": self.samples.get(key) and len(self.samples[key]) > 0,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init(args=[])
    observer = TfRuntimeObserver()
    try:
        traffic = observer.observe(args.duration)
        frames = observer.buffer.all_frames_as_yaml()
        imu_candidates = ("zed_imu_link", "imu_link", "zed_camera_imu_link")
        imu = next((name for name in imu_candidates if name in frames), None)
        required = [("map", "odom"), ("odom", "zed_camera_link"), ("zed_camera_link", "base_footprint"),
                    ("base_footprint", "lidar_link")]
        if imu:
            required.append(("base_footprint", imu))
        payload = {
            "schema_version": 1,
            "observer": {"duration_sec": args.duration, "node": observer.get_name(), "read_only": True},
            "transforms": [observer.lookup(parent, child) for parent, child in required],
            "observed_tf_traffic": traffic,
            "known_frames_yaml": frames,
            "imu_frame_selected": imu,
            "physical_camera_mount_expectation": {"base_to_zed_pitch_rad": 0.06981317007977318,
                                                    "zed_to_base_pitch_rad": -0.06981317007977318},
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps({"output": str(args.output), "transform_count": len(payload["transforms"]), "imu_frame": imu}))
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
