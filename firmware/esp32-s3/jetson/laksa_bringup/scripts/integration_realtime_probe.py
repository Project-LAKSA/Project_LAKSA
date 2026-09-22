#!/usr/bin/env python3
"""Read-only 10 Hz JSONL recorder for mapping/navigation integration."""

import json
import math
from pathlib import Path
import time

import psutil
import rclpy
from geometry_msgs.msg import Twist
from laksa_interfaces.msg import DriveCommand, VehicleState
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class IntegrationRealtimeProbe(Node):
    def __init__(self):
        super().__init__("laksa_integration_realtime_probe")
        self.declare_parameter("output_path", "/tmp/laksa_integration_realtime.jsonl")
        output = Path(str(self.get_parameter("output_path").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output.open("a", encoding="utf-8", buffering=1)
        self._latest = {}
        self._resources = {}
        self._last_resources = 0.0
        self._tf = Buffer(); self._tf_listener = TransformListener(self._tf, self)
        self.create_subscription(Odometry, "/laksa/odometry/fused", self._odom, 10)
        self.create_subscription(Twist, "/laksa/nav_cmd_vel", lambda m: self._twist("controller_candidate", m), 10)
        self.create_subscription(DriveCommand, "/laksa/autonomy_candidate_command", lambda m: self._drive("supervisor_candidate", m), 10)
        self.create_subscription(DriveCommand, "/laksa/command", lambda m: self._drive("supervisor_output", m), 10)
        self.create_subscription(Bool, "/laksa/brake", lambda m: self._set("brake", bool(m.data)), 10)
        self.create_subscription(Bool, "/laksa/autonomous_enabled", lambda m: self._set("autonomy_armed", bool(m.data)), 10)
        self.create_subscription(String, "/laksa/autonomy_health", lambda m: self._set("autonomy_health", m.data), 10)
        self.create_subscription(String, "/laksa/mapping/state", self._mapping, 10)
        self.create_subscription(VehicleState, "/laksa/state", self._vehicle, qos_profile_sensor_data)
        self.create_timer(0.1, self._record)

    def _set(self, name, value):
        self._latest[name] = value

    def _twist(self, name, message):
        self._latest[name] = {"vx": float(message.linear.x), "wz": float(message.angular.z)}

    def _drive(self, name, message):
        self._latest[name] = {
            "speed_mps": float(message.speed_mps),
            "steering_rad": float(message.steering_angle_rad),
            "brake": bool(message.brake),
        }

    def _odom(self, message):
        p = message.pose.pose.position; q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = message.header.stamp
        self._latest["pose"] = {
            "stamp": stamp.sec + stamp.nanosec * 1e-9,
            "x": float(p.x), "y": float(p.y), "yaw": yaw,
        }

    def _mapping(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._latest["mapping"] = {
            key: data.get(key) for key in (
                "state", "fusion_state", "fusion_reason_code", "rtab_processing_age_sec",
                "validated_scan_age_sec", "signal_ages_sec",
            )
        }

    def _vehicle(self, message):
        self._latest["actuator"] = {
            "requested_erpm": int(message.vesc.requested_erpm),
            "active_erpm": int(message.vesc.active_erpm),
            "measured_erpm": float(message.vesc.measured_erpm),
            "vesc_brake": bool(message.vesc.brake_active),
            "vesc_fault": int(message.vesc.fault_code),
            "steering_target_rad": float(message.steering_target_rad),
            "steering_current_rad": float(message.steering_current_rad),
        }

    def _sample_resources(self, now):
        if now - self._last_resources < 1.0:
            return
        self._last_resources = now
        wanted = ("zed", "rtabmap", "rgbd_sync", "cockpit", "controller_server", "planner_server", "drive_supervisor")
        result = {}
        for process in psutil.process_iter(("pid", "name", "cmdline", "memory_info")):
            try:
                command = " ".join(process.info.get("cmdline") or ())
                label = next((item for item in wanted if item in command), None)
                if label:
                    result[label] = {"pid": process.pid, "cpu_percent": process.cpu_percent(), "rss": process.info["memory_info"].rss}
            except (psutil.Error, KeyError):
                continue
        self._resources = result

    def _record(self):
        monotonic = time.monotonic(); self._sample_resources(monotonic)
        tf_age = None
        try:
            transform = self._tf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            stamp = transform.header.stamp
            tf_age = max(0.0, time.time() - (stamp.sec + stamp.nanosec * 1e-9))
        except TransformException:
            pass
        record = {
            "monotonic": monotonic, "utc": time.time(), "tf_age_sec": tf_age,
            **self._latest, "processes": self._resources,
        }
        self._stream.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")

    def destroy_node(self):
        self._stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = IntegrationRealtimeProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
