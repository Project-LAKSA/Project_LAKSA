#!/usr/bin/env python3

"""Interactive, subscriber-only room-scale longitudinal recorder."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from laksa_interfaces.msg import VehicleState, VescState
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, String


LABEL = "ROOM_SCALE_LONGITUDINAL_V1"
TRIALS = (
    {"number": 1, "direction": "FORWARD", "target_erpm": 900},
    {"number": 2, "direction": "REVERSE", "target_erpm": -900},
)


class RoomScaleRecorder(Node):
    def __init__(self, output_dir: Path) -> None:
        super().__init__("room_scale_longitudinal_recorder")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {}
        self._counts: Dict[str, int] = {}
        self._streams: Dict[str, Any] = {}
        self._session_start_utc = datetime.now(timezone.utc).isoformat()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        subscriptions = (
            ("/joy", Joy, 10),
            ("/laksa/state", VehicleState, qos_profile_sensor_data),
            ("/laksa/vesc/state", VescState, qos_profile_sensor_data),
            ("/laksa/imu/data", Imu, qos_profile_sensor_data),
            ("/laksa/autonomous_enabled", Bool, latched),
            ("/laksa/emergency_stop", Bool, latched),
        )
        for topic, message_type, qos in subscriptions:
            self.create_subscription(
                message_type,
                topic,
                lambda message, topic=topic: self._record_message(topic, message),
                qos,
            )

    def _record_message(self, topic: str, message: Any) -> None:
        now = time.monotonic()
        record = {
            "topic": topic,
            "received_monotonic": now,
            "received_utc": datetime.now(timezone.utc).isoformat(),
            "ros_stamp": _message_stamp(message),
            "data": _message_data(topic, message),
        }
        with self._lock:
            self._latest[topic] = record
            self._counts[topic] = self._counts.get(topic, 0) + 1
            stream = self._streams.get(topic)
            if stream is not None:
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")
                stream.flush()

    def latest(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest)

    def counts(self) -> Dict[str, int]:
        with self._lock:
            return dict(self._counts)

    def start_trial(self, trial_dir: Path) -> None:
        raw_dir = trial_dir / "raw_bag"
        raw_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self._streams = {
                topic: (raw_dir / _topic_filename(topic)).open(
                    "a", encoding="utf-8", buffering=1
                )
                for topic in (
                    "/joy",
                    "/laksa/state",
                    "/laksa/vesc/state",
                    "/laksa/imu/data",
                )
            }

    def stop_trial(self) -> None:
        with self._lock:
            streams = list(self._streams.values())
            self._streams = {}
        for stream in streams:
            stream.close()


def _topic_filename(topic: str) -> str:
    return topic.strip("/").replace("/", "_") + ".jsonl"


def _message_stamp(message: Any) -> Optional[Dict[str, int]]:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        stamp = getattr(message, "stamp", None)
    if stamp is None:
        return None
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def _message_data(topic: str, message: Any) -> Dict[str, Any]:
    if topic == "/joy":
        return {"axes": [float(value) for value in message.axes], "buttons": [int(value) for value in message.buttons]}
    if topic == "/laksa/imu/data":
        return {
            "linear_acceleration_m_s2": {
                "x": float(message.linear_acceleration.x),
                "y": float(message.linear_acceleration.y),
                "z": float(message.linear_acceleration.z),
            },
            "angular_velocity_rad_s": {
                "x": float(message.angular_velocity.x),
                "y": float(message.angular_velocity.y),
                "z": float(message.angular_velocity.z),
            },
        }
    if topic == "/laksa/state":
        vesc = message.vesc
        return {
            "requested_erpm": int(vesc.requested_erpm),
            "active_erpm": int(vesc.active_erpm),
            "measured_erpm": float(vesc.measured_erpm),
            "motor_current_a": float(vesc.motor_current_a),
            "input_current_a": float(vesc.input_current_a),
            "duty_cycle": float(vesc.duty_cycle),
            "input_voltage_v": float(vesc.input_voltage_v),
            "temp_motor_c": float(vesc.temp_motor_c),
            "temp_mosfet_c": float(vesc.temp_mosfet_c),
            "fault_code": int(vesc.fault_code),
            "command_fresh": bool(vesc.command_fresh),
            "telemetry_fresh": bool(vesc.telemetry_fresh),
            "steering_target_rad": float(message.steering_target_rad),
            "steering_current_rad": float(message.steering_current_rad),
            "linear_acceleration_m_s2": {
                "x": float(message.linear_acceleration_m_s2.x),
                "y": float(message.linear_acceleration_m_s2.y),
                "z": float(message.linear_acceleration_m_s2.z),
            },
        }
    if topic == "/laksa/vesc/state":
        return {
            "requested_erpm": int(message.requested_erpm),
            "active_erpm": int(message.active_erpm),
            "measured_erpm": float(message.measured_erpm),
            "motor_current_a": float(message.motor_current_a),
            "input_current_a": float(message.input_current_a),
            "duty_cycle": float(message.duty_cycle),
            "input_voltage_v": float(message.input_voltage_v),
            "temp_motor_c": float(message.temp_motor_c),
            "temp_mosfet_c": float(message.temp_mosfet_c),
            "fault_code": int(message.fault_code),
            "command_fresh": bool(message.command_fresh),
            "telemetry_fresh": bool(message.telemetry_fresh),
        }
    if topic in ("/laksa/autonomous_enabled", "/laksa/emergency_stop"):
        return {"data": bool(message.data)}
    return {}


def production_preflight(recorder: RoomScaleRecorder, wait_sec: float = 3.0) -> tuple[bool, str]:
    deadline = time.monotonic() + wait_sec
    while time.monotonic() < deadline:
        latest = recorder.latest()
        state = latest.get("/laksa/state", {}).get("data", {})
        vesc = latest.get("/laksa/vesc/state", {}).get("data", {})
        joy = latest.get("/joy")
        authority = latest.get("/laksa/autonomous_enabled", {}).get("data", {})
        if (
            joy is not None
            and state
            and vesc
            and latest.get("/laksa/imu/data")
            and authority.get("data") is False
            and latest.get("/laksa/emergency_stop", {}).get("data", {}).get("data") is False
            and state.get("fault_code") == 0
            and vesc.get("fault_code") == 0
            and state.get("telemetry_fresh") is True
            and vesc.get("telemetry_fresh") is True
            and state.get("command_fresh") is True
            and vesc.get("command_fresh") is True
        ):
            return True, "READY"
        time.sleep(0.1)
    missing = []
    latest = recorder.latest()
    for topic in ("/joy", "/laksa/state", "/laksa/vesc/state", "/laksa/imu/data"):
        if topic not in latest:
            missing.append(topic)
    state = latest.get("/laksa/state", {}).get("data", {})
    vesc = latest.get("/laksa/vesc/state", {}).get("data", {})
    if state.get("fault_code", 0) != 0 or vesc.get("fault_code", 0) != 0:
        missing.append("VESC fault_code must be 0")
    if state and state.get("telemetry_fresh") is not True:
        missing.append("/laksa/state telemetry_fresh")
    if vesc and vesc.get("telemetry_fresh") is not True:
        missing.append("/laksa/vesc/state telemetry_fresh")
    if state and state.get("command_fresh") is not True:
        missing.append("/laksa/state command_fresh")
    if vesc and vesc.get("command_fresh") is not True:
        missing.append("/laksa/vesc/state command_fresh")
    if latest.get("/laksa/autonomous_enabled", {}).get("data", {}).get("data") is not False:
        missing.append("autonomous_enabled must be false")
    if latest.get("/laksa/emergency_stop", {}).get("data", {}).get("data") is not False:
        missing.append("emergency_stop must be false; do not rearm automatically")
    return False, "; ".join(missing) or "required telemetry unavailable"


def prompt(message: str) -> str:
    try:
        return input(message).strip()
    except EOFError:
        return "QUIT"


def run_trial(recorder: RoomScaleRecorder, session_dir: Path, trial: Dict[str, Any]) -> bool:
    direction = trial["direction"]
    target = trial["target_erpm"]
    trial_dir = session_dir / "trials" / f"trial_{trial['number']:03d}_{direction.lower()}_900"
    trial_dir.mkdir(parents=True, exist_ok=True)
    print("=" * 60)
    print("LAKSA ROOM-SCALE LONGITUDINAL TEST")
    print(f"TEST {trial['number']} OF 2")
    print(f"Direction: {direction}")
    print(f"Target: {abs(target)} eRPM")
    print("\nPHYSICAL CHECK:")
    for item in (
        "LAKSA is on the floor.",
        "Steering is centered.",
        "Front points toward the clearest space.",
        "Rear also has clear space.",
        "Nothing fragile is near the vehicle.",
        "Xbox controller is ON and its light is solid.",
        "Your hand is on the controller.",
        "Throttle is neutral.",
        "Vehicle is stationary.",
    ):
        print(f"[ ] {item}")
    print("\nIMPORTANT: THIS SOFTWARE WILL NOT MOVE LAKSA.")
    print("You move LAKSA using the Xbox. Return to neutral immediately if space becomes limited.")
    print("Forward/reverse: left-stick Y. Preset: dashboard manual speed control at 900 eRPM.")
    answer = prompt("Press ENTER when ready, SKIP to skip, or QUIT to end: ").upper()
    if answer == "QUIT":
        return False
    if answer == "SKIP":
        _write_trial_metadata(
            trial_dir, trial, None, None, False, "skipped", None
        )
        return True

    recording_start_utc = datetime.now(timezone.utc).isoformat()
    recorder.start_trial(trial_dir)
    print("ZERO")
    time.sleep(2.0)
    print("3")
    time.sleep(1.0)
    print("2")
    time.sleep(1.0)
    print("1")
    time.sleep(1.0)
    print("GO - DRIVE")
    active_start = time.monotonic()
    print("DRIVE")
    print("0.5 s")
    time.sleep(0.5)
    print("1.0 s - RETURN TO NEUTRAL")
    user_neutral = prompt("Return to NEUTRAL now, then press ENTER: ")
    active_end = time.monotonic()
    print("NEUTRAL\nWAITING FOR SETTLE")
    time.sleep(2.0)
    print("TRIAL COMPLETE")
    user_valid = prompt("Was this trial clean? [Y/n] ").upper() not in ("N", "NO")
    notes = prompt("Notes (optional): ")
    recorder.stop_trial()
    _write_trial_metadata(
        trial_dir,
        trial,
        active_start,
        active_end,
        user_valid,
        notes,
        recording_start_utc,
    )
    return True


def _write_trial_metadata(
    trial_dir: Path,
    trial: Dict[str, Any],
    active_start: Optional[float],
    active_end: Optional[float],
    user_valid: bool,
    notes: str,
    recording_start_utc: Optional[str],
) -> None:
    now = datetime.now(timezone.utc).isoformat()
    metadata = {
        "target_erpm": trial["target_erpm"],
        "direction": trial["direction"],
        "repetition": 1,
        "requested_active_duration_sec": 1.0,
        "observed_active_duration_sec": (
            active_end - active_start
            if active_start is not None and active_end is not None
            else None
        ),
        "recording_start_utc": recording_start_utc or now,
        "recording_end_utc": datetime.now(timezone.utc).isoformat(),
        "user_valid": user_valid,
        "user_notes": notes,
        "test_label": LABEL,
    }
    (trial_dir / "metadata.yaml").write_text(
        "\n".join(f"{key}: {json.dumps(value)}" for key, value in metadata.items()) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Subscriber-only room-scale recorder.")
    parser.add_argument("--output-root", default="/home/ubuntu/laksa_vehicle_id")
    args = parser.parse_args()
    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_dir = Path(args.output_root) / session_id
    session_dir.mkdir(parents=True, exist_ok=True)

    print("ROOM_SCALE_LONGITUDINAL_V1")
    print("DRY RECORDER ONLY - THIS SOFTWARE WILL NOT MOVE LAKSA")
    print("No fused odometry is required for this first room-scale campaign.")
    rclpy.init()
    recorder = RoomScaleRecorder(session_dir)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(recorder)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        ready, reason = production_preflight(recorder)
        if not ready:
            print(f"PREFLIGHT BLOCKED: {reason}")
            return 2
        session_metadata = {
            "test_label": LABEL,
            "host": socket.gethostname(),
            "session_id": session_id,
            "production_environment_matched": True,
            "motion_authorized": False,
            "fused_odom_required": False,
            "trials_authorized": ["+900 eRPM forward", "-900 eRPM reverse"],
            "counts_at_preflight": recorder.counts(),
        }
        (session_dir / "session_metadata.yaml").write_text(
            "\n".join(f"{key}: {json.dumps(value)}" for key, value in session_metadata.items()) + "\n",
            encoding="utf-8",
        )
        for trial in TRIALS:
            if not run_trial(recorder, session_dir, trial):
                break
            if trial["number"] == 1:
                print("DO NOT reverse immediately.")
                print("Confirm LAKSA is stationary. You may manually reposition it now.")
                if prompt("Press ENTER when ready for REVERSE trial, or QUIT: ").upper() == "QUIT":
                    break
        print(f"SESSION COMPLETE: {session_dir}")
        return 0
    finally:
        recorder.stop_trial()
        executor.shutdown()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
