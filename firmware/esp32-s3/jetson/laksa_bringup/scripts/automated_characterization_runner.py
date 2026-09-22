#!/usr/bin/env python3
"""Supervised T10 physical-characterization runner.

This program is intentionally the *only* new motion-capable test tool.  It
publishes a bounded SI ``DriveCommand`` request to
``/laksa/characterization_request``; DriveSupervisor is still the sole
publisher of the canonical ``/laksa/command``.  It never talks to VESC, the
ESP32, PCA9685, a service, or an action server.
"""
from __future__ import annotations

import argparse
from collections import deque
import datetime as dt
import json
import math
import sqlite3
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml
from drivetrain_conversion import erpm_to_speed_mps, speed_mps_to_erpm
from interactive_ros import RosCallbackPump


STATES = ("IDLE", "PREFLIGHT", "ARMED", "PRE_SETTLE", "EXCITATION", "NEUTRAL", "POST_SETTLE", "COMPLETE", "ABORT")
ARM_SESSION_STATES = (
    "PREFLIGHT", "WAITING_FOR_ARM", "FINAL_PREFLIGHT", "ENABLE_GATE",
    "PRE_SETTLE", "EXCITATION_MOCK", "NEUTRAL", "POST_SETTLE",
    "DISABLE_GATE", "COMPLETE", "ABORT",
)


@dataclass
class TrialSchedule:
    pre_settle_sec: float
    excitation_sec: float
    post_settle_sec: float


@dataclass
class StationaryAssessment:
    """Explainable, traction-focused stationary result.

    IMU acceleration is deliberately diagnostic-only: a fixed gravity
    projection/bias says nothing about whether the driven wheels are stopped.
    """
    stationary: bool
    failing_predicate: str | None
    dwell_achieved_sec: float
    dwell_required_sec: float
    requested_erpm: float
    active_erpm: float
    measured_erpm: float
    max_abs_measured_erpm: float
    imu_diagnostic: dict


class StationaryDetector:
    """Require continuously healthy, zero-intent VESC evidence for a dwell.

    The detector is intentionally independent of ROS so its behavior can be
    tested against real-looking telemetry without creating any publisher.
    Xbox-neutrality remains a separate authority preflight predicate; it is
    not evidence that the traction system has stopped.
    """
    def __init__(self, erpm_threshold: float, dwell_sec: float):
        self.erpm_threshold = float(erpm_threshold)
        self.dwell_sec = float(dwell_sec)
        self._candidate_since_ns: int | None = None
        self.max_abs_measured_erpm = 0.0
        self.last_assessment: StationaryAssessment | None = None

    def update(self, now_ns: int, *, requested_erpm: float, active_erpm: float,
               measured_erpm: float, command_intent_zero: bool,
               telemetry_fresh: bool, command_fresh: bool, fault_code: int,
               imu_diagnostic: dict | None = None) -> StationaryAssessment:
        requested_erpm = float(requested_erpm); active_erpm = float(active_erpm)
        measured_erpm = float(measured_erpm)
        self.max_abs_measured_erpm = max(self.max_abs_measured_erpm, abs(measured_erpm))
        failing = None
        if int(fault_code): failing = "VESC_FAULT"
        elif not telemetry_fresh or not command_fresh: failing = "VESC_NOT_FRESH"
        elif not command_intent_zero: failing = "COMMAND_INTENT_NONZERO"
        elif abs(requested_erpm) > self.erpm_threshold: failing = "REQUESTED_ERPM_NONZERO"
        elif abs(active_erpm) > self.erpm_threshold: failing = "ACTIVE_ERPM_NONZERO"
        elif abs(measured_erpm) > self.erpm_threshold: failing = "MEASURED_ERPM_ABOVE_THRESHOLD"
        if failing:
            self._candidate_since_ns = None
            dwell = 0.0
        else:
            if self._candidate_since_ns is None: self._candidate_since_ns = int(now_ns)
            dwell = max(0.0, (int(now_ns) - self._candidate_since_ns) / 1e9)
        result = StationaryAssessment(
            stationary=failing is None and dwell >= self.dwell_sec,
            failing_predicate=failing,
            dwell_achieved_sec=dwell,
            dwell_required_sec=self.dwell_sec,
            requested_erpm=requested_erpm,
            active_erpm=active_erpm,
            measured_erpm=measured_erpm,
            max_abs_measured_erpm=self.max_abs_measured_erpm,
            imu_diagnostic=imu_diagnostic or {},
        )
        self.last_assessment = result
        return result

    def snapshot(self) -> dict:
        a = self.last_assessment
        if a is None: return {"final_decision": "NO_SAMPLE", "stationary_window_sec": self.dwell_sec}
        return {
            "final_decision": "STATIONARY_CONFIRMED" if a.stationary else "NOT_CONFIRMED",
            "stationary_window_sec": a.dwell_required_sec,
            "stationary_dwell_achieved_sec": a.dwell_achieved_sec,
            "failing_predicate": a.failing_predicate,
            "requested_erpm": a.requested_erpm, "active_erpm": a.active_erpm,
            "measured_erpm": a.measured_erpm,
            "max_abs_measured_erpm": a.max_abs_measured_erpm,
            "imu_diagnostic_only": a.imu_diagnostic,
        }


class TrialStateMachine:
    """Small deterministic state machine, separately testable without ROS."""
    def __init__(self, schedule: TrialSchedule):
        self.schedule, self.state, self.started_ns, self.abort_reason = schedule, "IDLE", 0, ""

    def start(self, now_ns: int):
        if self.state != "IDLE": raise RuntimeError("trial is already started")
        self.state, self.started_ns = "PREFLIGHT", now_ns

    def arm(self, now_ns: int):
        if self.state != "PREFLIGHT": raise RuntimeError("ARM is accepted only after preflight")
        self.state, self.started_ns = "ARMED", now_ns

    def tick(self, now_ns: int, preflight_ok: bool):
        if self.state == "ARMED":
            self.state, self.started_ns = "PRE_SETTLE", now_ns
        elif self.state == "PRE_SETTLE" and now_ns - self.started_ns >= int(self.schedule.pre_settle_sec * 1e9):
            self.state, self.started_ns = "EXCITATION", now_ns
        elif self.state == "EXCITATION" and now_ns - self.started_ns >= int(self.schedule.excitation_sec * 1e9):
            self.state, self.started_ns = "NEUTRAL", now_ns
        elif self.state == "NEUTRAL":
            self.state, self.started_ns = "POST_SETTLE", now_ns
        elif self.state == "POST_SETTLE" and now_ns - self.started_ns >= int(self.schedule.post_settle_sec * 1e9):
            self.state = "COMPLETE"
        return self.state

    def abort(self, reason: str):
        self.abort_reason, self.state = reason, "ABORT"

    @property
    def active(self): return self.state == "EXCITATION"


class ArmSessionStateMachine:
    """Separate unlimited human consent from the short machine watchdog.

    This class has no ROS dependency so the safety invariant is regression
    testable: the gate is never eligible to be enabled while waiting for ARM.
    """
    def __init__(self):
        self.state = "PREFLIGHT"
        self.gate_enabled = False
        self.abort_reason = ""

    def preflight_ready(self):
        if self.state != "PREFLIGHT": raise RuntimeError("preflight transition invalid")
        self.state = "WAITING_FOR_ARM"

    def arm(self):
        if self.state != "WAITING_FOR_ARM" or self.gate_enabled:
            raise RuntimeError("ARM requires disabled gate")
        self.state = "FINAL_PREFLIGHT"

    def final_preflight_ready(self):
        if self.state != "FINAL_PREFLIGHT": raise RuntimeError("final preflight transition invalid")
        self.state = "ENABLE_GATE"

    def gate_acknowledged(self):
        if self.state != "ENABLE_GATE": raise RuntimeError("gate acknowledgement invalid")
        self.gate_enabled = True
        self.state = "PRE_SETTLE"

    def excitation_mock(self):
        if self.state != "PRE_SETTLE" or not self.gate_enabled: raise RuntimeError("machine session not active")
        self.state = "EXCITATION_MOCK"

    def neutral(self):
        if self.state != "EXCITATION_MOCK": raise RuntimeError("neutral transition invalid")
        self.state = "NEUTRAL"

    def post_settle(self):
        if self.state != "NEUTRAL": raise RuntimeError("post-settle transition invalid")
        self.state = "POST_SETTLE"

    def disable(self):
        if self.state == "ABORT":
            self.gate_enabled = False
            return
        if self.state != "POST_SETTLE": raise RuntimeError("disable transition invalid")
        self.gate_enabled = False
        self.state = "DISABLE_GATE"

    def cleanup_acknowledged(self):
        if self.state != "DISABLE_GATE": raise RuntimeError("cleanup acknowledgement invalid")
        self.state = "COMPLETE"

    def abort(self, reason: str):
        self.abort_reason = reason
        self.state = "ABORT"


def erpm_to_mps(erpm: float, pole_pairs: float, gear_reduction: float, wheel_diameter_m: float) -> float:
    return erpm_to_speed_mps(erpm, pole_pairs, gear_reduction, wheel_diameter_m)


def utc_stamp(): return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def mock_arm_lifecycle() -> dict:
    """Offline regression harness: no ROS, gate, or DriveCommand is created."""
    machine=ArmSessionStateMachine(); states=[machine.state]
    machine.preflight_ready();states.append(machine.state)
    machine.arm();states.append(machine.state)
    machine.final_preflight_ready();states.append(machine.state)
    machine.gate_acknowledged();states.append(machine.state)
    machine.excitation_mock();states.append(machine.state)
    machine.neutral();states.append(machine.state)
    machine.post_settle();states.append(machine.state)
    machine.disable();states.append(machine.state)
    machine.cleanup_acknowledged();states.append(machine.state)
    return {"status":"MOCK_COMPLETE","states":states,"ros_created":False,"drive_command_published":False}


def write_jsonl(stream, row):
    stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n"); stream.flush()


def register_dataset(lab_root: Path, trial_dir: Path, metadata: dict, status: str):
    """Backward-compatible append to the existing A045 registry, never raw data."""
    if not lab_root.exists(): return None
    db_path = lab_root / "lab_outputs" / "experiments.sqlite"; db_path.parent.mkdir(parents=True, exist_ok=True)
    rid = f"physical-{metadata['session_id']}"
    with sqlite3.connect(db_path, timeout=30) as db:
        db.execute("CREATE TABLE IF NOT EXISTS experiments(run_id TEXT PRIMARY KEY,kind TEXT NOT NULL,status TEXT NOT NULL,metadata TEXT NOT NULL)")
        db.execute("INSERT OR REPLACE INTO experiments VALUES(?,?,?,?)", (rid, "physical_characterization", status, json.dumps({**metadata, "raw_artifact_dir": str(trial_dir)}, sort_keys=True)))
    return rid


def main(argv=None):
    parser = argparse.ArgumentParser(description="One independently armed supervised T10 trial.")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config" / "characterization_tests.yaml")
    parser.add_argument("--output-root", type=Path, default=Path("/home/ubuntu/laksa_vehicle_id"))
    parser.add_argument("--lab-root", type=Path, default=Path("/home/ubuntu/src/Project_LAKSA/firmware/esp32-s3/tools/a045_closed_loop_autonomy"))
    parser.add_argument("--direction", choices=("forward", "reverse"), required=True)
    parser.add_argument("--dry-run", action="store_true", help="Validate configuration only; publish nothing.")
    parser.add_argument("--mock-arm", action="store_true", help="Offline ARM/gate state-machine verification; no ROS or motion.")
    parser.add_argument("--stationary-preflight", action="store_true", help="Read-only live stationary detector check; publishes nothing.")
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text())
    t10 = config["t10"]; target_erpm = int(t10["target_erpm_equivalent"]) * (1 if args.direction == "forward" else -1)
    schedule = TrialSchedule(*(float(t10[key]) for key in ("pre_settle_sec", "excitation_sec", "post_settle_sec")))
    if args.mock_arm:
        print(json.dumps(mock_arm_lifecycle(),indent=2)); return 0
    if args.dry_run and not args.stationary_preflight:
        print(json.dumps({"status": "DRY_RUN_OK", "target_erpm_equivalent": target_erpm, "schedule": schedule.__dict__, "publisher": "/laksa/characterization_request", "canonical_output_publisher": False}, indent=2)); return 0

    import rclpy
    from laksa_interfaces.msg import DriveCommand, VehicleState, VescState
    from sensor_msgs.msg import Joy, Imu
    from std_msgs.msg import Bool
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    class Runner(Node):
        def __init__(self, trial_dir: Path):
            super().__init__("t10_automated_characterization")
            self.pub = self.create_publisher(DriveCommand, "/laksa/characterization_request", 10)
            gate_qos=QoSProfile(depth=1);gate_qos.reliability=ReliabilityPolicy.RELIABLE;gate_qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
            self.gate_pub=self.create_publisher(Bool,"/laksa/characterization_enable",gate_qos)
            self.last_state = None; self.last_vesc = None; self.last_command = None
            self.last_state_ns = 0; self.last_vesc_ns = 0; self.last_command_ns = 0; self.last_joy_ns = 0
            self.estop = False; self.autonomous = False; self.gate_enabled = None
            self.imu_samples = deque(maxlen=200)
            self.last_preflight_assessment = None
            self.callback_pump = False
            self.stationary_detector = StationaryDetector(
                t10["stationary_erpm_threshold"], t10["stationary_dwell_sec"])
            self.logs = (trial_dir / "raw_bag" / "automated_trace.jsonl").open("w", encoding="utf-8")
            self.create_subscription(VehicleState, "/laksa/state", self.state_cb, qos_profile_sensor_data)
            self.create_subscription(VescState, "/laksa/vesc/state", self.vesc_cb, qos_profile_sensor_data)
            self.create_subscription(DriveCommand, "/laksa/command", self.command_cb, 10)
            self.create_subscription(Joy, "/joy", self.joy_cb, 10)
            self.create_subscription(Imu, "/laksa/imu/data", self.imu_cb, qos_profile_sensor_data)
            self.create_subscription(Bool, "/laksa/emergency_stop", lambda m: setattr(self, "estop", bool(m.data)), 10)
            self.create_subscription(Bool, "/laksa/autonomous_enabled", lambda m: setattr(self, "autonomous", bool(m.data)), 10)
            self.create_subscription(Bool, "/laksa/characterization_enabled", lambda m: setattr(self, "gate_enabled", bool(m.data)), gate_qos)
        def now(self): return time.monotonic_ns()
        def state_cb(self, m):
            self.last_state, self.last_state_ns = m, self.now(); v = m.vesc
            write_jsonl(self.logs, {"topic": "/laksa/state", "received_monotonic": self.last_state_ns / 1e9, "data": {"requested_erpm": int(v.requested_erpm), "active_erpm": int(v.active_erpm), "measured_erpm": float(v.measured_erpm), "motor_current_a": float(v.motor_current_a), "input_current_a": float(v.input_current_a), "input_voltage_v": float(v.input_voltage_v), "duty_cycle": float(v.duty_cycle), "fault_code": int(v.fault_code), "telemetry_fresh": bool(v.telemetry_fresh), "command_fresh": bool(v.command_fresh), "steering_target_rad": float(m.steering_target_rad), "steering_current_rad": float(m.steering_current_rad)}})
        def vesc_cb(self, v):
            self.last_vesc, self.last_vesc_ns = v, self.now()
            write_jsonl(self.logs, {"topic": "/laksa/vesc/state", "received_monotonic": self.last_vesc_ns / 1e9, "data": {"requested_erpm": int(v.requested_erpm), "active_erpm": int(v.active_erpm), "measured_erpm": float(v.measured_erpm), "motor_current_a": float(v.motor_current_a), "input_current_a": float(v.input_current_a), "input_voltage_v": float(v.input_voltage_v), "duty_cycle": float(v.duty_cycle), "fault_code": int(v.fault_code), "telemetry_fresh": bool(v.telemetry_fresh), "command_fresh": bool(v.command_fresh)}})
        def command_cb(self, m):
            self.last_command, self.last_command_ns = m, self.now()
            write_jsonl(self.logs, {"topic": "/laksa/command", "received_monotonic": self.last_command_ns/1e9, "data": {"speed_mps": float(m.speed_mps), "steering_angle_rad": float(m.steering_angle_rad), "brake": bool(m.brake)}})
        def joy_cb(self, _m): self.last_joy_ns = self.now()
        def imu_cb(self, m):
            now=self.now(); x=float(m.linear_acceleration.x)
            angular=(float(m.angular_velocity.x),float(m.angular_velocity.y),float(m.angular_velocity.z))
            self.imu_samples.append((now,x,math.sqrt(sum(v*v for v in angular))))
            write_jsonl(self.logs, {"topic": "/laksa/imu/data", "received_monotonic": now/1e9, "data": {"linear_acceleration_m_s2": {"x": x, "y": float(m.linear_acceleration.y), "z": float(m.linear_acceleration.z)}, "angular_velocity_rad_s": {"x": angular[0], "y": angular[1], "z": angular[2]}}})
        def close(self): self.logs.close()
        def imu_diagnostic(self):
            recent=[sample for sample in self.imu_samples if self.now()-sample[0] <= 1_000_000_000]
            if not recent: return {"sample_count":0,"used_for_stationary_gate":False}
            xs=[s[1] for s in recent]; omegas=[s[2] for s in recent]
            return {"sample_count":len(recent),"accel_x_min_m_s2":min(xs),"accel_x_max_m_s2":max(xs),"accel_x_range_m_s2":max(xs)-min(xs),"angular_speed_max_rad_s":max(omegas),"used_for_stationary_gate":False}
        def safety_preflight(self):
            now = self.now(); m = self.last_state
            if not m or not self.last_vesc: return False, "NO_VEHICLE_OR_VESC_STATE"
            if now-self.last_state_ns > 500_000_000 or now-self.last_vesc_ns > 500_000_000: return False, "STATE_STALE"
            if now-self.last_joy_ns > 500_000_000: return False, "XBOX_STALE"
            if self.estop: return False, "ESTOP_TRUE"
            if self.autonomous: return False, "AUTONOMY_CONFLICT"
            v=self.last_vesc
            if int(v.fault_code): return False, "VESC_FAULT"
            if not bool(v.telemetry_fresh) or not bool(v.command_fresh): return False, "VESC_NOT_FRESH"
            if abs(float(m.steering_current_rad)) > float(t10["steering_center_tolerance_rad"]): return False, "STEERING_NOT_CENTERED"
            return True, "READY"
        def stationary(self):
            v=self.last_vesc
            if v is None:
                return None
            # ``requested_erpm`` and ``active_erpm`` are the authoritative
            # applied traction-intent observables.  /laksa/command is a useful
            # corroborating volatile topic, but its first sample is not a safe
            # prerequisite for deciding that a *powered drivetrain* is at
            # rest.  When it is observed, it must itself be zero.
            command_zero=(abs(float(v.requested_erpm)) <= self.stationary_detector.erpm_threshold and abs(float(v.active_erpm)) <= self.stationary_detector.erpm_threshold)
            if self.last_command is not None:
                command_zero=command_zero and abs(float(self.last_command.speed_mps)) <= 1e-6
            diagnostic=self.imu_diagnostic()
            diagnostic["canonical_command_observed"] = self.last_command is not None
            diagnostic["canonical_command_speed_mps"] = (float(self.last_command.speed_mps) if self.last_command is not None else None)
            diagnostic["traction_intent_source"] = "laksa_command_and_vesc" if self.last_command is not None else "vesc_requested_active"
            return self.stationary_detector.update(
                self.now(), requested_erpm=float(v.requested_erpm), active_erpm=float(v.active_erpm),
                measured_erpm=float(v.measured_erpm), command_intent_zero=command_zero,
                telemetry_fresh=bool(v.telemetry_fresh), command_fresh=bool(v.command_fresh),
                fault_code=int(v.fault_code),
                imu_diagnostic=diagnostic)
        def preflight(self):
            ok,reason=self.safety_preflight()
            if not ok:return False,reason,None
            assessment=self.stationary()
            if assessment is None:return False,"NO_VEHICLE_OR_VESC_STATE",None
            return assessment.stationary, ("READY" if assessment.stationary else assessment.failing_predicate or "STATIONARY_DWELL_INCOMPLETE"), assessment
        def publish(self, speed, brake=False):
            msg=DriveCommand();msg.speed_mps=float(speed);msg.steering_angle_rad=0.0;msg.brake=bool(brake);self.pub.publish(msg)
        def set_gate(self, enabled):
            msg=Bool();msg.data=bool(enabled);self.gate_pub.publish(msg)
        def refresh_preflight(self, timeout_sec=2.0):
            deadline=time.monotonic()+timeout_sec; reason="NO_VEHICLE_OR_VESC_STATE"
            while time.monotonic()<deadline:
                if self.callback_pump: time.sleep(.05)
                else: rclpy.spin_once(self,timeout_sec=.05)
                ok,reason,assessment=self.preflight()
                self.last_preflight_assessment=assessment
                if ok:return True,"READY"
            return False,reason
        def stationary_failure_message(self):
            a=self.last_preflight_assessment
            if a is None:return "[FAIL] Vehicle not stationary: no valid VESC stationary sample"
            return ("[FAIL] Vehicle not stationary: "
                    f"measured_erpm = {a.measured_erpm:.1f}; requested_erpm = {a.requested_erpm:.1f}; "
                    f"active_erpm = {a.active_erpm:.1f}; stationary dwell achieved = "
                    f"{a.dwell_achieved_sec:.2f} / {a.dwell_required_sec:.2f} sec; "
                    f"predicate = {a.failing_predicate or 'DWELL_INCOMPLETE'}")
        def wait_gate(self, enabled, timeout_sec=10.0):
            deadline=time.monotonic()+timeout_sec
            while time.monotonic()<deadline:
                self.set_gate(enabled)
                if self.callback_pump: time.sleep(.05)
                else: rclpy.spin_once(self,timeout_sec=.05)
                if self.gate_enabled is enabled:return True
            return self.gate_enabled is enabled

    if args.stationary_preflight:
        rclpy.init(); check_dir=Path("/tmp")/f"laksa_stationary_preflight_{utc_stamp()}"; (check_dir/"raw_bag").mkdir(parents=True,exist_ok=True); node=Runner(check_dir); pump=RosCallbackPump(node);node.callback_pump=True;pump.start()
        try:
            # A freshly restarted DDS graph can require several seconds to
            # discover its existing telemetry writers.  This is read-only and
            # does not relax the actual stationary dwell or any safety bound.
            deadline=time.monotonic()+max(5.0,float(t10["stationary_dwell_sec"])+1.0); assessment=None
            while time.monotonic()<deadline:
                time.sleep(.05); assessment=node.stationary()
                if assessment is not None and assessment.stationary: break
            authority_ok,authority_reason=node.safety_preflight()
            report={"status":"STATIONARY_CONFIRMED" if assessment and assessment.stationary else "STATIONARY_NOT_CONFIRMED","traction_stationary_only":True,"authority_preflight_status":"READY" if authority_ok else authority_reason,"stationary_detector":node.stationary_detector.snapshot(),"motion_command_published":False,"gate_message_published":False}
            print(json.dumps(report,indent=2,sort_keys=True));
            if not assessment or not assessment.stationary: print(node.stationary_failure_message())
            return 0 if assessment and assessment.stationary else 2
        finally:
            pump.stop();node.close();node.destroy_node();rclpy.shutdown()

    session_id = f"T10_AUTOMATED_V1_{utc_stamp()}"; trial_dir=args.output_root/session_id/"trials"/f"trial_01_{args.direction}_{abs(target_erpm)}"; (trial_dir/"raw_bag").mkdir(parents=True,exist_ok=True)
    conversion={"pole_pairs":2.0,"gear_reduction":11.82,"wheel_diameter_m":.109,"source":"drivetrain_conversion.py; mirrors ESP32 speed_to_erpm"}; target_mps=erpm_to_speed_mps(target_erpm,2.0,11.82,.109); conversion["resolved_speed_mps"]=target_mps; conversion["round_trip_erpm"]=speed_mps_to_erpm(target_mps,2.0,11.82,.109)
    meta = {"schema_version":"laksa-t10-automated-v1","test_id":"T10_AUTOMATED_V1","session_id":session_id,"direction":args.direction,"target_erpm":target_erpm,"target_domain":"SI speed_mps through /laksa/characterization_request","conversion":conversion,"manual_timing_contaminated":False,"operator_valid":False,"resolved_config":config,"created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),"clock_provenance":"time.monotonic_ns receive/publish timestamps"}
    rclpy.init(); node=Runner(trial_dir); pump=RosCallbackPump(node);node.callback_pump=True;pump.start(); machine=TrialStateMachine(schedule); session=ArmSessionStateMachine(); events=[]
    def event(kind, **data):
        row={"event":kind,"monotonic_ns":time.monotonic_ns(),**data};events.append(row);write_jsonl(node.logs,{"topic":"/laksa/characterization_event","received_monotonic":row["monotonic_ns"]/1e9,"data":row})
    result=2; analysis_path=trial_dir.parents[1]/"analysis.json"; machine_active=False; motion_request_started=False; cleanup_ok=False
    try:
        event("RUNNER_STARTED", target_erpm=target_erpm)
        # The ordinary preflight and the entire human wait occur with the gate
        # disabled and with no DriveCommand publication.
        if not node.wait_gate(False): raise RuntimeError("GATE_DISABLE_ACK_TIMEOUT")
        event("GATE_DISABLED_CONFIRMED")
        ok,reason=node.refresh_preflight()
        if not ok:
            if node.last_preflight_assessment is not None: print(node.stationary_failure_message())
            raise RuntimeError(f"INITIAL_PREFLIGHT_{reason}")
        session.preflight_ready();event("STATE_WAITING_FOR_ARM", gate_enabled=False)
        print(f"READY: {args.direction} {target_erpm} eRPM-equivalent; {schedule.excitation_sec}s maximum. Gate is disabled. Type ARM to begin one trial, SKIP, or QUIT.")
        choice=input("ARM> ").strip().upper()
        if choice in ("QUIT","SKIP"):
            machine.abort("OPERATOR_QUIT_BEFORE_ARM" if choice=="QUIT" else "OPERATOR_SKIP_BEFORE_ARM")
            session.abort(machine.abort_reason);event("ABORT",reason=machine.abort_reason,gate_enabled=False);result=0
        elif choice != "ARM":
            machine.abort("ARM_NOT_CONFIRMED");session.abort(machine.abort_reason);event("ABORT",reason=machine.abort_reason,gate_enabled=False)
        else:
            session.arm();event("ARM_CONFIRMED",gate_enabled=False)
            # Input blocks callbacks, so reacquire fresh samples only now.
            ok,reason=node.refresh_preflight()
            if not ok:
                if node.last_preflight_assessment is not None: print(node.stationary_failure_message())
                machine.abort(f"FINAL_PREFLIGHT_{reason}");session.abort(machine.abort_reason);event("ABORT",reason=machine.abort_reason,gate_enabled=False)
            else:
                session.final_preflight_ready();event("STATE_ENABLE_GATE")
                if not node.wait_gate(True):
                    machine.abort("ENABLE_ACK_TIMEOUT");session.abort(machine.abort_reason);event("ABORT",reason=machine.abort_reason)
                else:
                    session.gate_acknowledged();machine.start(time.monotonic_ns());machine.arm(time.monotonic_ns());machine.tick(time.monotonic_ns(),True);machine_active=True
                    event("GATE_ENABLED_ACK",gate_enabled=True);event("STATE_PRE_SETTLE")
                    event("TEST_TARGET_RESOLVED",speed_mps=target_mps,target_erpm=target_erpm,conversion=conversion);event("REQUEST_INTENT", speed_mps=target_mps, target_erpm_equivalent=target_erpm)
                    while machine.state not in ("COMPLETE","ABORT"):
                        time.sleep(.02)
                        # A nonzero eRPM is expected only in EXCITATION.  The
                        # neutral/post-settle phases must first send zero and
                        # then *observe* a continuous stationary dwell; they
                        # must never be rejected for the prior sample.
                        if machine.state == "PRE_SETTLE":
                            ok,reason,assessment=node.preflight()
                        else:
                            ok,reason=node.safety_preflight();assessment=None
                            if machine.state in ("NEUTRAL","POST_SETTLE") and ok:
                                assessment=node.stationary()
                        if machine.state == "EXCITATION":
                            # During excitation expected eRPM is nonzero; only
                            # health/authority predicates apply.
                            ok,reason=node.safety_preflight();reason="SAFETY_STATE_LOST" if not ok else "READY"
                        if not ok:
                            machine.abort(reason);session.abort(reason);event("ABORT",reason=reason);break
                        prior=machine.state;machine.tick(time.monotonic_ns(),True)
                        if machine.state != prior:event(f"STATE_{machine.state}")
                        requested_speed=target_mps if machine.active else 0.0;node.publish(requested_speed,False);motion_request_started=True
                        if machine.active and not any(e["event"]=="CHARACTERIZATION_REQUEST_NONZERO" for e in events):event("CHARACTERIZATION_REQUEST_NONZERO",speed_mps=requested_speed)
                        if machine.state == "COMPLETE":
                            # The immediately preceding POST_SETTLE loop has
                            # already observed zero intent and low eRPM.  Keep
                            # the final evidence explicit in the artifact.
                            assessment=node.stationary()
                            event("STATIONARY_FINAL", **node.stationary_detector.snapshot())
                            if not assessment.stationary:
                                reason=f"NEUTRAL_NOT_STATIONARY_{assessment.failing_predicate or 'DWELL_INCOMPLETE'}"
                                machine.abort(reason);session.abort(reason);event("ABORT",reason=reason);break
                        time.sleep(1.0/float(t10["request_heartbeat_hz"]))
                    if machine.state=="COMPLETE": result=0
    except KeyboardInterrupt:
        machine.abort("OPERATOR_INTERRUPT");session.abort(machine.abort_reason);event("ABORT",reason=machine.abort_reason)
    except Exception as error:
        machine.abort(str(error));session.abort(machine.abort_reason);event("ABORT",reason=machine.abort_reason)
    finally:
        if machine_active:
            try:node.publish(0.0,True);event("NEUTRAL_FINAL")
            except Exception:pass
        try:
            cleanup_ok=node.wait_gate(False);event("GATE_DISABLED_CLEANUP",acknowledged=cleanup_ok)
        except Exception: cleanup_ok=False
        meta["events"]=events;meta["stationary_detector"]=node.stationary_detector.snapshot();meta["status"]="COMPLETE" if machine.state=="COMPLETE" else ("QUIT" if machine.abort_reason.startswith("OPERATOR_") else "ABORTED");meta["abort_reason"]=machine.abort_reason or None;meta["operator_valid"]=machine.state=="COMPLETE";meta["gate_cleanup_acknowledged"]=cleanup_ok;meta["motion_request_started"]=motion_request_started;meta["physical_motion_expected"]=bool(motion_request_started);meta["analysis_available"]=False
        (trial_dir/"metadata.yaml").write_text(yaml.safe_dump(meta,sort_keys=False))
        if machine.state=="COMPLETE":
            from analyze_room_scale_longitudinal import session_report, markdown_report
            analysis=session_report(trial_dir.parents[1]);analysis_path.write_text(json.dumps(analysis,indent=2,sort_keys=True)+"\n");analysis_path.with_suffix(".md").write_text(markdown_report(analysis));meta["analysis_available"]=True
        rid=register_dataset(args.lab_root,trial_dir,meta,"NEEDS_REVIEW" if machine.state=="COMPLETE" else "ABORTED")
        print(json.dumps({"status":meta["status"],"abort_reason":meta["abort_reason"],"trial_dir":str(trial_dir),"analysis":str(analysis_path) if analysis_path.exists() else None,"registry_run_id":rid,"gate_cleanup_acknowledged":cleanup_ok},indent=2))
        pump.stop();node.close();node.destroy_node();rclpy.shutdown()
    return result

if __name__ == "__main__": raise SystemExit(main())
