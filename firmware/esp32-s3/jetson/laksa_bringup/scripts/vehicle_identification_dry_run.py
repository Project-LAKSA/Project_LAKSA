#!/usr/bin/env python3

"""Read-only longitudinal vehicle-identification preflight."""

from __future__ import annotations

import argparse
import json
import os
import socket
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import rclpy
from geometry_msgs.msg import Twist
from laksa_interfaces.msg import VehicleState, VescState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, Joy, LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


@dataclass(frozen=True)
class TopicSpec:
    msg_type: Any
    critical: bool
    expected_hz: Optional[float] = None
    minimum_hz: Optional[float] = None
    freshness_sec: Optional[float] = 2.0


INTERFACE_TOPICS: Dict[str, TopicSpec] = {
    "/joy": TopicSpec(Joy, True, 20.0, 10.0, 1.0),
    "/laksa/state": TopicSpec(VehicleState, True, 10.0, 5.0, 1.0),
    "/laksa/vesc/state": TopicSpec(VescState, True, 10.0, 5.0, 1.0),
    "/laksa/imu/data": TopicSpec(Imu, True, 50.0, 25.0, 1.0),
    "/laksa/autonomous_enabled": TopicSpec(Bool, True, None, None, None),
    "/laksa/autonomy_health": TopicSpec(String, True, None, None, 5.0),
    "/laksa/emergency_stop": TopicSpec(Bool, True, None, None, None),
}

REQUIRED_FOR_PHYSICAL_ID: Dict[str, TopicSpec] = {
    "/laksa/odometry/fused": TopicSpec(Odometry, False, None, 1.0, 2.0),
}

OPTIONAL_TOPICS: Dict[str, TopicSpec] = {
    "/tf": TopicSpec(TFMessage, False, None, None, 5.0),
    "/tf_static": TopicSpec(TFMessage, False, None, None, 10.0),
    "/map": TopicSpec(OccupancyGrid, False, None, None, 10.0),
    "/laksa/lidar/scan_validated": TopicSpec(
        LaserScan, False, None, None, 2.0
    ),
    "/zed/zed_node/point_cloud/cloud_registered": TopicSpec(
        PointCloud2, False, None, None, 2.0
    ),
    "/laksa/nav_cmd_vel": TopicSpec(Twist, False, None, None, 2.0),
    "/laksa/lidar_cruise_cmd_vel": TopicSpec(Twist, False, None, None, 2.0),
}

VEHICLE_GEOMETRY = {
    "wheelbase_m": 0.324,
    "wheel_diameter_m": 0.109,
    "gear_reduction": 11.82,
    "motor_pole_pairs": 2.0,
    "steering_center_deg": 100,
    "steering_pca_channel": 7,
    "max_steering_rad": 0.523,
    "left_road_wheel_limit_rad": 0.523,
    "right_road_wheel_limit_rad": 0.288,
}

VESC_BASELINE = {
    "motor_type": "FOC sensorless",
    "ramp_erpm_per_s": 3000,
    "max_abs_erpm": 3000,
    "command_timeout_ms": 500,
    "telemetry_interval_ms": 200,
    "uart_baud": 115200,
}

TEST_MATRIX = [
    ("900 eRPM", "3 repetitions"),
    ("1500 eRPM", "3 repetitions"),
    ("2500 eRPM", "3 repetitions"),
    ("4000 eRPM", "DISABLED / NOT YET AUTHORIZED"),
]


class TopicObservation:
    def __init__(self, topic: str, spec: TopicSpec) -> None:
        self.topic = topic
        self.spec = spec
        self.msg_type = _message_type_name(spec.msg_type)
        self.count = 0
        self.first_receive: Optional[float] = None
        self.last_receive: Optional[float] = None
        self.intervals = []
        self.last_state: Dict[str, Any] = {}

    def record(self, now: float) -> None:
        if self.last_receive is not None:
            self.intervals.append(now - self.last_receive)
        if self.first_receive is None:
            self.first_receive = now
        self.last_receive = now
        self.count += 1

    def summary(self, end_time: float) -> Dict[str, Any]:
        span = (
            self.last_receive - self.first_receive
            if self.first_receive is not None and self.last_receive is not None
            else 0.0
        )
        rate = (self.count - 1) / span if self.count > 1 and span > 0.0 else 0.0
        age = (
            end_time - self.last_receive
            if self.last_receive is not None
            else None
        )
        fresh = (
            age is not None
            and (
                self.spec.freshness_sec is None
                or age <= self.spec.freshness_sec
            )
        )
        adequate = (
            self.spec.minimum_hz is None
            or (self.count > 1 and rate >= self.spec.minimum_hz)
        )
        if self.count == 0 or not fresh or not adequate:
            status = "FAIL" if self.spec.critical else "WARN"
        else:
            status = "PASS"
        return {
            "type": self.msg_type,
            "critical": self.spec.critical,
            "expected_rate_hz": self.spec.expected_hz,
            "minimum_rate_hz": self.spec.minimum_hz,
            "count": self.count,
            "first_receive_monotonic": self.first_receive,
            "last_receive_monotonic": self.last_receive,
            "observed_rate_hz": rate,
            "latest_age_sec": age,
            "fresh": fresh,
            "adequate_rate": adequate,
            "status": status,
            "latest_content": self.last_state,
        }


def _message_type_name(msg_type: Any) -> str:
    return msg_type.__module__.replace(".", "/") + "/" + msg_type.__name__


def _finite_or_none(value: Any) -> Any:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if value == value and abs(value) != float("inf") else None


def _vehicle_content(message: Any) -> Dict[str, Any]:
    vesc = message.vesc if isinstance(message, VehicleState) else message
    steering_target = (
        message.steering_target_rad if isinstance(message, VehicleState) else None
    )
    steering_current = (
        message.steering_current_rad if isinstance(message, VehicleState) else None
    )
    return {
        "requested_erpm": int(vesc.requested_erpm),
        "measured_erpm": _finite_or_none(vesc.measured_erpm),
        "motor_current_a": _finite_or_none(vesc.motor_current_a),
        "input_current_a": _finite_or_none(vesc.input_current_a),
        "duty_cycle": _finite_or_none(vesc.duty_cycle),
        "input_voltage_v": _finite_or_none(vesc.input_voltage_v),
        "temp_motor_c": _finite_or_none(vesc.temp_motor_c),
        "temp_mosfet_c": _finite_or_none(vesc.temp_mosfet_c),
        "fault_code": int(vesc.fault_code),
        "command_fresh": bool(vesc.command_fresh),
        "telemetry_fresh": bool(vesc.telemetry_fresh),
        "telemetry_age_ms": int(vesc.telemetry_age_ms),
        "steering_target_rad": _finite_or_none(steering_target),
        "steering_current_rad": _finite_or_none(steering_current),
    }


def production_environment() -> Dict[str, Any]:
    expected = {
        "ros_domain_id": "0",
        "ros_localhost_only": "1",
        "rmw_implementation": "rmw_cyclonedds_cpp",
        "cyclonedds_uri": "/etc/laksa/cyclonedds.xml",
    }
    cyclonedds_uri = os.environ.get("CYCLONEDDS_URI", "")
    # The canonical launcher exports a standards-compliant file URI while old
    # production notes use the same local path without the URI scheme.  They
    # identify the same DDS configuration, so a read-only preflight must not
    # invent an environment failure from that spelling difference.
    if cyclonedds_uri.startswith("file://"):
        cyclonedds_uri = cyclonedds_uri[len("file://"):]
    actual = {
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        "cyclonedds_uri": cyclonedds_uri,
    }
    return {**actual, "matched": actual == expected}


def subscription_qos(topic: str) -> QoSProfile:
    if topic == "/joy":
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        return qos
    if topic in (
        "/laksa/autonomous_enabled",
        "/laksa/autonomy_health",
        "/laksa/emergency_stop",
    ):
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        return qos
    return qos_profile_sensor_data


def evaluate_preflight(
    summaries: Dict[str, Dict[str, Any]],
    latest_content: Dict[str, Dict[str, Any]],
    environment_matched: bool = True,
) -> Dict[str, Any]:
    interface_failures = []
    warnings = []
    for topic, summary in summaries.items():
        if summary["critical"] and summary["status"] == "FAIL":
            interface_failures.append(topic)
        elif summary["status"] == "WARN":
            warnings.append(topic)

    state = latest_content.get("/laksa/state", {})
    vesc_state = latest_content.get("/laksa/vesc/state", {})
    emergency = latest_content.get("/laksa/emergency_stop", {}).get("data")
    autonomous = latest_content.get("/laksa/autonomous_enabled", {}).get("data")

    if not environment_matched:
        interface_failures.append("production DDS environment mismatch")
    if state.get("fault_code", 0) != 0 or vesc_state.get("fault_code", 0) != 0:
        interface_failures.append("VESC fault reported")
    if state.get("telemetry_fresh") is not True or vesc_state.get("telemetry_fresh") is not True:
        interface_failures.append("VESC telemetry stale")
    if state.get("command_fresh") is not True or vesc_state.get("command_fresh") is not True:
        interface_failures.append("VESC command freshness failure")
    if not isinstance(emergency, bool):
        interface_failures.append("emergency-stop state unavailable")
    if not isinstance(autonomous, bool):
        interface_failures.append("manual authority state unavailable")

    communication_health = "PASS" if not interface_failures else "FAIL"
    fused_odom = summaries.get("/laksa/odometry/fused", {})
    fused_odom_status = (
        "PASS"
        if fused_odom.get("status") == "PASS"
        else "MISSING_PUBLISHER"
        if fused_odom.get("count", 0) == 0
        else "STALE"
    )
    motion_block_reasons = []
    if autonomous is True:
        motion_block_reasons.append("AUTONOMY_ACTIVE")
    if emergency is True:
        motion_block_reasons.append("EMERGENCY_STOP")
    if latest_content.get("/laksa/autonomy_health", {}).get("data") == "ODOM_STALE":
        motion_block_reasons.append("ODOM_STALE")
    motion_authorized = (
        communication_health == "PASS"
        and fused_odom_status == "PASS"
        and autonomous is False
        and emergency is False
    )

    return {
        "communication_health": communication_health,
        "motion_authorized": motion_authorized,
        "motion_block_reason": "/".join(dict.fromkeys(motion_block_reasons)) or None,
        "fused_odom_status": fused_odom_status,
        "interface_result": (
            "INTERFACE_CHECK_PASS"
            if communication_health == "PASS"
            else "INTERFACE_CHECK_FAIL"
        ),
        "vehicle_id_result": (
            "VEHICLE_ID_FAIL"
            if communication_health != "PASS"
            else "VEHICLE_ID_READY"
            if motion_authorized
            else "VEHICLE_ID_BLOCKED_ODOM"
            if fused_odom_status != "PASS"
            else "VEHICLE_ID_FAIL"
        ),
        "failures": sorted(set(interface_failures)),
        "warnings": sorted(set(warnings)),
    }


def _yaml_scalar(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    return json.dumps(str(value))


def _yaml_lines(value: Any, indent: int = 0) -> Iterable[str]:
    prefix = " " * indent
    if isinstance(value, dict):
        for key, item in value.items():
            if isinstance(item, (dict, list)):
                yield f"{prefix}{key}:"
                yield from _yaml_lines(item, indent + 2)
            else:
                yield f"{prefix}{key}: {_yaml_scalar(item)}"
    elif isinstance(value, list):
        for item in value:
            if isinstance(item, (dict, list)):
                yield f"{prefix}-"
                yield from _yaml_lines(item, indent + 2)
            else:
                yield f"{prefix}- {_yaml_scalar(item)}"


def write_session_files(
    output_dir: Path,
    summaries: Dict[str, Dict[str, Any]],
    preflight: Dict[str, Any],
    latest_content: Dict[str, Dict[str, Any]],
    end_time: float,
    mode: str,
    environment: Dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    session_id = output_dir.name
    metadata = {
        "utc_session_id": session_id,
        "host": socket.gethostname(),
        "mode": mode,
        "production_environment_matched": environment["matched"],
        "rmw_implementation": environment["rmw_implementation"],
        "ros_domain_id": environment["ros_domain_id"],
        "ros_localhost_only": environment["ros_localhost_only"],
        "cyclonedds_uri": environment["cyclonedds_uri"],
        "vehicle_geometry": VEHICLE_GEOMETRY,
        "vesc_baseline": VESC_BASELINE,
        "dry_run": True,
        "motion_authorized": False,
    }
    runtime = {
        "interface_result": preflight["interface_result"],
        "vehicle_id_result": preflight["vehicle_id_result"],
        "communication_health": preflight["communication_health"],
        "motion_authorized": preflight["motion_authorized"],
        "motion_block_reason": preflight["motion_block_reason"],
        "fused_odom_status": preflight["fused_odom_status"],
        "topics": summaries,
        "interface_topics": list(INTERFACE_TOPICS),
        "required_for_physical_id": list(REQUIRED_FOR_PHYSICAL_ID),
        "optional_topics": list(OPTIONAL_TOPICS),
        "latest_vehicle_content": latest_content,
    }
    (output_dir / "metadata.yaml").write_text(
        "\n".join(_yaml_lines(metadata)) + "\n", encoding="utf-8"
    )
    (output_dir / "runtime_interface.yaml").write_text(
        "\n".join(_yaml_lines(runtime)) + "\n", encoding="utf-8"
    )
    lines = [
        "LAKSA LONGITUDINAL IDENTIFICATION",
        "",
        "DRY RUN ONLY - NO COMMANDS WILL BE SENT",
        "",
        f"INTERFACE RESULT: {preflight['interface_result']}",
        f"VEHICLE-ID RESULT: {preflight['vehicle_id_result']}",
        f"COMMUNICATION HEALTH: {preflight['communication_health']}",
        f"MOTION AUTHORIZED: {preflight['motion_authorized']}",
        f"FUSED ODOM STATUS: {preflight['fused_odom_status']}",
        f"Session: {session_id}",
        f"End monotonic time: {end_time:.3f}",
        "",
        "Failures:",
        *[f"- {item}" for item in preflight["failures"] or ["NONE"]],
        "Warnings:",
        *[f"- {item}" for item in preflight["warnings"] or ["NONE"]],
        "",
        "Future trial timing:",
        "PRE-IDLE: 2 s",
        "TARGET HOLD: 4 s",
        "COMMAND ZERO",
        "COAST: 3 s",
        "POST-IDLE: 2 s",
    ]
    (output_dir / "preflight.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


class VehicleIdentificationDryRun(Node):
    def __init__(
        self,
        duration_sec: float,
        output_dir: Optional[Path],
        interface_check: bool,
    ) -> None:
        super().__init__("vehicle_identification_dry_run")
        self._duration_sec = float(duration_sec)
        self._start_monotonic = time.monotonic()
        self._output_dir = output_dir
        self._interface_check = interface_check
        self._mode = "interface_check" if interface_check else "vehicle_id_preflight"
        self._environment = production_environment()
        self._observations = {
            topic: TopicObservation(topic, spec)
            for topic, spec in {
                **INTERFACE_TOPICS,
                **REQUIRED_FOR_PHYSICAL_ID,
                **OPTIONAL_TOPICS,
            }.items()
        }
        self.result_code = 1
        self.report = None

        for topic, spec in {
            **INTERFACE_TOPICS,
            **REQUIRED_FOR_PHYSICAL_ID,
            **OPTIONAL_TOPICS,
        }.items():
            self.create_subscription(
                spec.msg_type,
                topic,
                lambda message, topic=topic: self._topic_callback(topic, message),
                subscription_qos(topic),
            )
        self._timer = self.create_timer(1.0, self._status_tick)

    def _topic_callback(self, topic: str, message: Any) -> None:
        observation = self._observations[topic]
        now = time.monotonic()
        observation.record(now)
        if topic in ("/laksa/state", "/laksa/vesc/state"):
            observation.last_state = _vehicle_content(message)
        elif topic in (
            "/laksa/autonomous_enabled",
            "/laksa/emergency_stop",
        ):
            observation.last_state = {"data": bool(message.data)}
        elif topic == "/laksa/autonomy_health":
            observation.last_state = {"data": str(message.data)}

    def _status_tick(self) -> None:
        now = time.monotonic()
        if now - self._start_monotonic >= self._duration_sec:
            self._finish(now)
            return
        critical_received = sum(
            self._observations[topic].count > 0 for topic in INTERFACE_TOPICS
        )
        self.get_logger().info(
            f"Dry-run heartbeat: critical topics received "
            f"{critical_received}/{len(INTERFACE_TOPICS)}"
        )

    def _finish(self, end_time: float) -> None:
        summaries = {
            topic: observation.summary(end_time)
            for topic, observation in self._observations.items()
        }
        latest_content = {
            topic: observation.last_state
            for topic, observation in self._observations.items()
            if observation.last_state
        }
        preflight = evaluate_preflight(
            summaries,
            latest_content,
            environment_matched=self._environment["matched"],
        )
        self.report = {
            "result": (
                preflight["interface_result"]
                if self._interface_check
                else preflight["vehicle_id_result"]
            ),
            "dry_run": True,
            "motion_authorized": False,
            "no_motion_commands": True,
            "interface_topics": {
                topic: summaries[topic] for topic in INTERFACE_TOPICS
            },
            "required_for_physical_id": {
                topic: summaries[topic] for topic in REQUIRED_FOR_PHYSICAL_ID
            },
            "optional_topics": {
                topic: summaries[topic] for topic in OPTIONAL_TOPICS
            },
            "preflight": preflight,
            "latest_vehicle_content": latest_content,
        }
        if self._output_dir is not None:
            write_session_files(
                self._output_dir,
                summaries,
                preflight,
                latest_content,
                end_time,
                self._mode,
                self._environment,
            )
        print(json.dumps(self.report, indent=2, sort_keys=True))
        result = (
            preflight["interface_result"]
            if self._interface_check
            else preflight["vehicle_id_result"]
        )
        self.result_code = 0 if result in (
            "INTERFACE_CHECK_PASS",
            "VEHICLE_ID_READY",
            "VEHICLE_ID_BLOCKED_ODOM",
        ) else 1
        self._timer.cancel()
        raise SystemExit(self.result_code)


def _print_test_matrix() -> None:
    print("LAKSA LONGITUDINAL IDENTIFICATION")
    print("DRY RUN ONLY - NO COMMANDS WILL BE SENT")
    for target, repetitions in TEST_MATRIX:
        print(f"{target}\n  {repetitions}")
    print("\nFuture trial timing:")
    print("  PRE-IDLE: 2 s")
    print("  TARGET HOLD: 4 s")
    print("  COMMAND ZERO")
    print("  COAST: 3 s")
    print("  POST-IDLE: 2 s")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only longitudinal identification preflight."
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument(
        "--interface-check",
        action="store_true",
        help="Check the production DDS/control telemetry plane without requiring odometry.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Metadata directory; default is /home/ubuntu/laksa_vehicle_id/<UTC_SESSION>.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.interface_check:
        _print_test_matrix()
    else:
        print("INTERFACE CHECK: READ ONLY - NO COMMANDS WILL BE SENT")
    utc_session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = Path(args.output_dir) if args.output_dir else Path(
        "/home/ubuntu/laksa_vehicle_id"
    ) / utc_session
    print(f"Metadata output: {output_dir}")
    print("Read-only subscriptions only; no publishers, clients, or actions created.")

    rclpy.init()
    node = VehicleIdentificationDryRun(
        args.duration,
        output_dir,
        args.interface_check,
    )
    try:
        rclpy.spin(node)
    except (
        KeyboardInterrupt,
        rclpy.executors.ExternalShutdownException,
        SystemExit,
    ):
        pass
    finally:
        result_code = node.result_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
