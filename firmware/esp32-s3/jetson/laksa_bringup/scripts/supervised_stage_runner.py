#!/usr/bin/env python3
"""Run one bounded characterization stage only through ``drive_supervisor``.

Every terminal outcome writes a JSON terminal artifact. An ABORT artifact is
not a physical-trial analysis: it explicitly says whether any non-neutral
request or characterization authority was observed.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import select
import signal
import sys
import time
from pathlib import Path

import yaml

from batch_characterization_manifest import validate
from batch_session_lease import FileLease
from local_campaign_core import assess_observation, required_topics
from supervised_stage_protocols import Engine, build

TESTS = ("T21A", "T21B", "T22", "T23", "T24", "T25", "T26", "T27")


def stamp():
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def write(stream, row):
    stream.write(json.dumps(row, sort_keys=True) + "\n")
    stream.flush()


def write_json(path: Path, value) -> None:
    """Write a derived artifact; callers never rewrite raw.jsonl."""
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def terminal_artifact(*, test_id, variant, status, abort_reason, started_utc,
                      events, safety, authority, motion, cleanup):
    """Machine-readable outcome for both COMPLETE and pre-motion ABORT."""
    trial_ran = bool(motion["non_neutral_request_publish_count"])
    return {
        "schema_version": "laksa-supervised-stage-terminal-v1",
        "artifact_type": "TERMINAL_OUTCOME",
        "test_id": test_id,
        "variant": variant,
        "status": status,
        "abort_reason": abort_reason,
        "timestamps": {"started_utc": started_utc, "finished_utc": utc_now()},
        "trial_analysis": {
            "status": "NOT_RUN" if not trial_ran else "RAW_ONLY_PENDING_DERIVED_ANALYSIS",
            "reason": "preflight_or_arm_abort_before_non_neutral_request" if not trial_ran else None,
        },
        "safety": safety,
        "characterization_authority": authority,
        "motion": motion,
        "cleanup": cleanup,
        "events": events,
        "publisher_ownership": {
            "motion_request": "/laksa/characterization_request",
            "canonical_command": "drive_supervisor",
        },
        "raw_immutable": True,
        "registry_status": "PENDING_HOST_INGEST",
    }


def _interrupt_to_cleanup(_signum, _frame):
    """Make terminal/SSH signals execute the runner's fail-safe finally path."""
    raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", choices=TESTS, required=True)
    parser.add_argument("--variant", required=True)
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parents[1] / "config" / "supervised_characterization_protocols.yaml")
    parser.add_argument("--output-root", type=Path, default=Path("/home/ubuntu/laksa_vehicle_id"))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--session-authorization", help="exact hash-bound batch authorization phrase")
    parser.add_argument("--batch-manifest", type=Path, help="immutable JSON manifest for this batch")
    parser.add_argument("--lease-file", type=Path, help="host-renewed lease; expiry fails closed")
    parser.add_argument("--rehearsal-only", action="store_true")
    parser.add_argument("--auto-arm", action="store_true", help="local finite campaign only; never reads stdin")
    parser.add_argument("--preflight-only", action="store_true", help="observe required channels and exit with gate false; never arms")
    args = parser.parse_args(argv)
    cfg = yaml.safe_load(args.config.read_text())
    phases = build(args.test, args.variant, cfg)
    engine = Engine(phases)
    batch_mode = any((args.session_authorization, args.batch_manifest, args.lease_file))
    if batch_mode:
        if not all((args.session_authorization, args.batch_manifest, args.lease_file)):
            parser.error("batch mode requires --session-authorization, --batch-manifest, and --lease-file")
        manifest = json.loads(args.batch_manifest.read_text())
        validate(manifest, args.session_authorization)
        matching = [row for row in manifest.get("trials", [])
                    if row.get("stage") == args.test and row.get("variant") == args.variant]
        if len(matching) != 1:
            parser.error("trial is not uniquely authorized by the batch manifest")
        if matching[0].get("phases") != [phase.__dict__ for phase in phases]:
            parser.error("local protocol does not exactly match authorized manifest")
        lease = FileLease(args.lease_file, manifest["lease_max_age_s"])
        lease.require()
    else:
        lease = None
    if args.dry_run:
        print(json.dumps({
            "status": "DRY_RUN_OK", "test_id": args.test, "variant": args.variant,
            "phases": [item.__dict__ for item in phases],
            "motion_publishers": ["/laksa/characterization_request"],
            "canonical_command_publisher": "drive_supervisor",
            "gate_topic": "/laksa/characterization_enable", "no_motion": True,
            "batch_mode": batch_mode,
        }, indent=2))
        return 0

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from laksa_interfaces.msg import DriveCommand, VehicleState, VescState
    from sensor_msgs.msg import Imu, Joy
    from std_msgs.msg import Bool

    class Watch(Node):
        def __init__(self, raw):
            super().__init__("supervised_" + args.test.lower())
            self.raw = raw
            self.state = self.vesc = None
            self.state_ns = self.vesc_ns = self.joy_ns = 0
            self.estop = self.auto = self.manual = False
            self.gate = None
            self.stationary_since = None
            self.last_safety = None
            self.request_publish_count = 0
            self.non_neutral_request_publish_count = 0
            self.gate_enable_publish_count = 0
            self.gate_enable_acknowledged = False
            self.observations = {topic: [] for topic in required_topics(args.test)}
            self.req = self.create_publisher(DriveCommand, "/laksa/characterization_request", 10)
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.gate_pub = self.create_publisher(Bool, "/laksa/characterization_enable", qos)
            self.create_subscription(VehicleState, "/laksa/state", self.scb, qos_profile_sensor_data)
            self.create_subscription(VescState, "/laksa/vesc/state", self.vcb, qos_profile_sensor_data)
            self.create_subscription(DriveCommand, "/laksa/command", self.command_cb, 10)
            self.create_subscription(Imu, "/laksa/imu/data", self.imu_cb, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/laksa/odometry/fused", self.odom_cb, qos_profile_sensor_data)
            self.create_subscription(Joy, "/joy", self.jcb, 10)
            self.create_subscription(Bool, "/laksa/emergency_stop", lambda msg: setattr(self, "estop", bool(msg.data)), 10)
            self.create_subscription(Bool, "/laksa/autonomous_enabled", lambda msg: setattr(self, "auto", bool(msg.data)), 10)
            self.create_subscription(Bool, "/laksa/characterization_enabled", self.gate_cb, qos)

        def now(self):
            return time.monotonic_ns()

        def log(self, topic, data):
            write(self.raw, {"topic": topic, "monotonic_ns": self.now(), "data": data})

        def observed(self, topic):
            if topic in self.observations:
                self.observations[topic].append(self.now())

        def scb(self, msg):
            self.observed("/laksa/state")
            self.state = msg
            self.state_ns = self.now()
            self.log("/laksa/state", {"steering_target_rad": msg.steering_target_rad, "steering_current_rad": msg.steering_current_rad})

        def vcb(self, msg):
            self.observed("/laksa/vesc/state")
            self.vesc = msg
            self.vesc_ns = self.now()
            self.log("/laksa/vesc/state", {"requested_erpm": msg.requested_erpm, "active_erpm": msg.active_erpm,
                                              "measured_erpm": msg.measured_erpm, "fault_code": msg.fault_code,
                                              "telemetry_fresh": msg.telemetry_fresh, "command_fresh": msg.command_fresh,
                                              "input_voltage_v": msg.input_voltage_v, "motor_current_a": msg.motor_current_a,
                                              "input_current_a": msg.input_current_a, "duty_cycle": msg.duty_cycle})

        def command_cb(self, msg):
            self.log("/laksa/command", {"speed_mps": msg.speed_mps, "steering_rad": msg.steering_angle_rad, "brake": msg.brake})

        def imu_cb(self, msg):
            self.observed("/laksa/imu/data")
            self.log("/laksa/imu/data", {
                "angular_velocity_rad_s": {"x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z},
                "linear_acceleration_m_s2": {"x": msg.linear_acceleration.x, "y": msg.linear_acceleration.y, "z": msg.linear_acceleration.z},
                "orientation_quaternion": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w},
                "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            })

        def odom_cb(self, msg):
            self.observed("/laksa/odometry/fused")
            self.log("/laksa/odometry/fused", {"x_m": msg.pose.pose.position.x, "y_m": msg.pose.pose.position.y,
                                                  "speed_mps": msg.twist.twist.linear.x,
                                                  "yaw_rate_rad_s": msg.twist.twist.angular.z,
                                                  "orientation_quaternion": {"x": msg.pose.pose.orientation.x,
                                                                              "y": msg.pose.pose.orientation.y,
                                                                              "z": msg.pose.pose.orientation.z,
                                                                              "w": msg.pose.pose.orientation.w},
                                                  "frame_id": msg.header.frame_id,
                                                  "child_frame_id": msg.child_frame_id,
                                                  "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9})

        def jcb(self, msg):
            self.observed("/joy")
            self.joy_ns = self.now()
            self.manual = any(abs(axis) > cfg["manual_axis_deadband"] for axis in msg.axes)
            self.log("/joy", {"manual_override": self.manual})

        def gate_cb(self, msg):
            self.gate = bool(msg.data)
            if self.gate:
                self.gate_enable_acknowledged = True
            self.log("/laksa/characterization_enabled", {"enabled": self.gate})

        def _age_s(self, timestamp_ns, now_ns):
            return None if not timestamp_ns else (now_ns - timestamp_ns) / 1e9

        def safety_snapshot(self, stationary=False):
            now = self.now()
            vesc = self.vesc
            ages = {"state": self._age_s(self.state_ns, now), "vesc": self._age_s(self.vesc_ns, now), "joy": self._age_s(self.joy_ns, now)}
            stale = [name for name, age in ages.items() if age is None or age > cfg["freshness_sec"]]
            blockers = []
            if self.state is None or vesc is None:
                blockers.append("MISSING_STATE")
            if stale:
                blockers.append("STALE_TOPIC_" + "_".join(stale).upper())
            if self.estop:
                blockers.append("ESTOP")
            if self.auto:
                blockers.append("AUTONOMY_CONFLICT")
            if self.manual:
                blockers.append("XBOX_MANUAL_OVERRIDE")
            if lease is not None and not lease.valid():
                blockers.append("SESSION_LEASE_EXPIRED")
            vesc_state = None if vesc is None else {"fault_code": int(vesc.fault_code), "telemetry_fresh": bool(vesc.telemetry_fresh),
                                                      "command_fresh": bool(vesc.command_fresh), "requested_erpm": float(vesc.requested_erpm),
                                                      "active_erpm": float(vesc.active_erpm), "measured_erpm": float(vesc.measured_erpm)}
            if vesc_state and (vesc_state["fault_code"] or not vesc_state["telemetry_fresh"] or not vesc_state["command_fresh"]):
                blockers.append("VESC_FAULT_OR_STALE")
            if stationary and vesc_state and any(abs(vesc_state[key]) > cfg["stationary_erpm_threshold"] for key in ("requested_erpm", "active_erpm", "measured_erpm")):
                self.stationary_since = None
                blockers.append("NOT_STATIONARY")
            if stationary and not blockers:
                self.stationary_since = self.stationary_since or now
                if now - self.stationary_since < cfg["stationary_dwell_sec"] * 1e9:
                    blockers.append("STATIONARY_DWELL")
            elif not stationary:
                self.stationary_since = None
            snapshot = {"checked_utc": utc_now(), "stationary_required": bool(stationary), "topic_age_s": ages,
                        "estop": self.estop, "autonomy_enabled": self.auto, "manual_override": self.manual,
                        "gate_observed": self.gate, "vesc": vesc_state, "blocking_conditions": blockers}
            self.last_safety = snapshot
            return snapshot

        def safe(self, stationary=False):
            snapshot = self.safety_snapshot(stationary=stationary)
            return (not snapshot["blocking_conditions"], snapshot["blocking_conditions"][0] if snapshot["blocking_conditions"] else "READY")

        def mandatory_topics_ok(self):
            return assess_observation(required_topics(args.test), self.observations,
                                      freshness_s=float(cfg["freshness_sec"]), now_ns=self.now())

        def send(self, phase):
            msg = DriveCommand()
            msg.speed_mps = float(phase.speed_mps)
            msg.steering_angle_rad = float(phase.steering_rad)
            msg.brake = bool(phase.brake)
            self.req.publish(msg)
            self.request_publish_count += 1
            if msg.speed_mps or msg.steering_angle_rad or msg.brake:
                self.non_neutral_request_publish_count += 1
            self.log("/laksa/characterization_request", {"speed_mps": msg.speed_mps, "steering_rad": msg.steering_angle_rad, "brake": msg.brake})

        def gate_send(self, value):
            msg = Bool()
            msg.data = bool(value)
            self.gate_pub.publish(msg)
            if msg.data:
                self.gate_enable_publish_count += 1

    root = args.output_root / f"{args.test}_{stamp()}"
    root.mkdir(parents=True)
    raw = (root / "raw.jsonl").open("w")
    started_utc = utc_now()
    rclpy.init()
    node = Watch(raw)
    result = 2
    events = []
    cleanup = {"neutral_request_attempted": False, "gate_disable_attempted": False, "gate_disabled_acknowledged": False, "errors": []}

    def event(name, **kwargs):
        row = {"event": name, "monotonic_ns": time.monotonic_ns(), "utc": utc_now(), **kwargs}
        events.append(row)
        write(raw, {"topic": "/laksa/characterization_event", "data": row})

    original_handlers = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    for signum in original_handlers:
        signal.signal(signum, _interrupt_to_cleanup)
    try:
        # Allow the already-running Xbox/telemetry publishers to reacquire
        # after a supervisor reload without weakening the freshness bound.
        deadline = time.monotonic() + 15.0
        ok = False
        reason = "PREFLIGHT_TIMEOUT"
        topics_ok = False
        topic_failures, topic_report = [], {}
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.gate_send(False)
            ok, reason = node.safe(stationary=True)
            topics_ok, topic_failures, topic_report = node.mandatory_topics_ok()
            # Do not exit as soon as the stationary dwell has elapsed.  The
            # mandatory observation window has its own sample-count contract.
            if ok and topics_ok:
                break
        if not ok:
            event("PREFLIGHT_FAILED", reason=reason, safety=node.last_safety)
            raise RuntimeError("PREFLIGHT_" + reason)
        if not topics_ok:
            event("MANDATORY_TOPIC_PREFLIGHT_FAILED", failures=topic_failures, topics=topic_report)
            raise RuntimeError("MANDATORY_TOPIC_PREFLIGHT_" + topic_failures[0])
        engine.preflight_ok()
        event("PREFLIGHT_READY", safety=node.last_safety, mandatory_topics=topic_report)
        if args.preflight_only:
            engine.abort("PREMOTION_PREFLIGHT_ONLY")
            event("PREMOTION_PREFLIGHT_COMPLETE", mandatory_topics=topic_report)
            result = 0
            return result
        if args.rehearsal_only:
            if args.test != "T22":
                parser.error("--rehearsal-only is only valid for T22")
            # Keep the ROS executor alive during the visible START countdown.
            # A blocking sleep lets /joy, state, and VESC freshness expire
            # between the first preflight and the final pre-motion check.
            countdown_deadline = time.monotonic() + 3.0
            announced = None
            while True:
                rclpy.spin_once(node, timeout_sec=0.05)
                node.gate_send(False)
                remaining = max(1, int(countdown_deadline - time.monotonic() + 0.999))
                if remaining != announced:
                    print(f"T22_REHEARSAL_READY: START in {remaining}", flush=True)
                    announced = remaining
                if time.monotonic() >= countdown_deadline:
                    break
            choice = "ARM"
        else:
            choice = "ARM" if (batch_mode or args.auto_arm) else input("ARM to begin one bounded supervised trial; otherwise QUIT: ").strip().upper()
        if choice != "ARM":
            engine.abort("OPERATOR_NOT_ARMED")
            event("ARM_DECLINED")
        else:
            ok, reason = node.safe(stationary=True)
            if not ok:
                engine.abort("FINAL_PREFLIGHT_" + reason)
                event("FINAL_PREFLIGHT_FAILED", reason=reason, safety=node.last_safety)
            else:
                topics_ok, topic_failures, topic_report = node.mandatory_topics_ok()
                if not topics_ok:
                    engine.abort("FINAL_MANDATORY_TOPIC_" + topic_failures[0])
                    event("FINAL_MANDATORY_TOPIC_PREFLIGHT_FAILED", failures=topic_failures, topics=topic_report)
                else:
                    event("FINAL_MANDATORY_TOPIC_PREFLIGHT_READY", topics=topic_report)
            if engine.state != "ABORT":
                engine.arm()
                until = time.monotonic() + 3.0
                while time.monotonic() < until and node.gate is not True:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    node.gate_send(True)
                if node.gate is not True:
                    engine.abort("GATE_ACK_TIMEOUT")
                    event("GATE_ENABLE_FAILED", observed_gate=node.gate)
                else:
                    engine.enabled(time.monotonic())
                    event("GATE_ENABLED")
                    while engine.state == "PHASE":
                        rclpy.spin_once(node, timeout_sec=0.01)
                        ok, reason = node.safe()
                        if not ok:
                            engine.abort(reason)
                            event("SAFETY_ABORT", reason=reason, safety=node.last_safety)
                            break
                        if args.test == "T22" and not args.auto_arm and select.select([sys.stdin], [], [], 0)[0] and input().strip().upper() == "FINISH":
                            engine.finish()
                            event("HUMAN_FINISH_EVENT")
                        node.gate_send(True)
                        node.send(engine.request)
                        engine.tick(time.monotonic(), True)
                        time.sleep(1.0 / cfg["request_heartbeat_hz"])
                    node.send(type(engine.request)("NEUTRAL", 0))
                    event("NEUTRAL_SENT")
                    settle_until = time.monotonic() + 8.0
                    while engine.state == "SETTLE" and time.monotonic() < settle_until:
                        rclpy.spin_once(node, timeout_sec=0.05)
                        node.gate_send(True)
                        node.send(type(engine.request)("NEUTRAL", 0))
                        ok, reason = node.safe(stationary=True)
                        if ok:
                            engine.settled()
                    if engine.state == "SETTLE":
                        engine.abort("STATIONARY_TIMEOUT")
                        event("SETTLE_FAILED", reason="STATIONARY_TIMEOUT", safety=node.last_safety)
                    result = 0 if engine.state == "COMPLETE" else 2
    except KeyboardInterrupt:
        engine.abort("OPERATOR_INTERRUPT")
        event("INTERRUPTED")
    except Exception as error:
        engine.abort(str(error))
    finally:
        try:
            node.send(type(engine.request)("NEUTRAL", 0))
            cleanup["neutral_request_attempted"] = True
            node.gate_send(False)
            cleanup["gate_disable_attempted"] = True
            deadline = time.monotonic() + 0.75
            while time.monotonic() < deadline and node.gate is not False:
                rclpy.spin_once(node, timeout_sec=0.05)
                node.gate_send(False)
            cleanup["gate_disabled_acknowledged"] = node.gate is False
            event("FAILSAFE_NEUTRAL_GATE_DISABLED", cleanup=cleanup)
        except Exception as error:
            cleanup["errors"].append(str(error))
        safety = node.last_safety or node.safety_snapshot(stationary=False)
        authority = {"enable_publish_count": node.gate_enable_publish_count, "ever_enabled_acknowledged": node.gate_enable_acknowledged,
                     "final_gate_observed": node.gate}
        motion = {"request_publish_count": node.request_publish_count, "non_neutral_request_publish_count": node.non_neutral_request_publish_count,
                  "non_neutral_motion_requested": bool(node.non_neutral_request_publish_count)}
        topics_ok, topic_failures, topic_report = node.mandatory_topics_ok()
        report = terminal_artifact(test_id=args.test, variant=args.variant, status=engine.state, abort_reason=engine.abort_reason,
                                   started_utc=started_utc, events=events, safety=safety, authority=authority, motion=motion, cleanup=cleanup)
        report["mandatory_topic_data_quality"] = {"classification": "DATA_QUALITY_PASS" if topics_ok else "DATA_QUALITY_FAIL",
                                                   "failures": topic_failures, "topics": topic_report}
        write_json(root / "metadata.json", report)
        write_json(root / "analysis.json", report)
        raw.close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        finally:
            for signum, handler in original_handlers.items():
                signal.signal(signum, handler)
        print(json.dumps({"status": engine.state, "session": str(root), "registry_run_id": "HOST_INGEST_REQUIRED"}, indent=2))
        if args.rehearsal_only:
            print("T22_REHEARSAL_END_VEHICLE_STOPPED", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
