"""Owned, fail-fast launcher for gated C1.2d closed-loop qualification."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Callable, Iterable, Mapping

from .owned_process_session import OwnedProcessSession
from .ros_domain import (
    RosDomainError,
    RosDomainLease,
    allocate_isolated_domain,
    claim_explicit_domain,
)
from .runtime_preflight import (
    RuntimePreflightError,
    validate_ackermann_runtime,
    validate_f1tenth_gym_runtime,
)


F1TENTH_GYM_SHA = "bdaec1420c3b0f103858d289866d0d4e2e597c30"
DT_S = 0.01
MAX_SPEED_MPS = 1.0
SHORT_HORIZON_STEPS = 50


REQUIRED_READY_NODES = frozenset(
    {
        "/c1/nav2_raceline",
        "/c1/mppi_lockstep_host",
        "/c1/nav2_ackermann_adapter",
    }
)
REQUIRED_READY_TOPICS = frozenset(
    {
        "/c1/nav2_path",
        "/c1/nav2_cmd_vel",
        "/c1/drive_request",
    }
)


class QualificationHarnessError(RuntimeError):
    """A C1 qualification prerequisite or critical process failed."""


def _lines(command: list[str], *, env: Mapping[str, str] | None = None) -> set[str]:
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        env=None if env is None else dict(env),
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise QualificationHarnessError(f"probe failed ({' '.join(command)}): {detail}")
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def runtime_graph_snapshot(
    env: Mapping[str, str] | None = None,
) -> tuple[set[str], set[str]]:
    return (
        _lines(
            ["ros2", "node", "list", "--no-daemon", "--spin-time", "3.0"], env=env
        ),
        _lines(
            ["ros2", "topic", "list", "--no-daemon", "--spin-time", "3.0"], env=env
        ),
    )


def domain_has_active_graph(domain_id: int) -> bool:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["ROS_LOCALHOST_ONLY"] = "1"
    nodes = _lines(
        ["ros2", "node", "list", "--no-daemon", "--spin-time", "3.0"], env=env
    )
    return bool(nodes)


def wait_for_ready(
    session: OwnedProcessSession,
    *,
    timeout_s: float,
    probe: Callable[[], tuple[set[str], set[str]]] = runtime_graph_snapshot,
    poll_s: float = 0.1,
) -> dict[str, list[str]]:
    """Require two consecutive complete graph snapshots while launch is alive."""

    deadline = time.monotonic() + timeout_s
    consecutive = 0
    last_nodes: set[str] = set()
    last_topics: set[str] = set()
    while time.monotonic() < deadline:
        returncode = session.process.poll()
        if returncode is not None:
            raise QualificationHarnessError(
                f"critical runtime exited before ready with status {returncode}"
            )
        last_nodes, last_topics = probe()
        complete = REQUIRED_READY_NODES <= last_nodes and REQUIRED_READY_TOPICS <= last_topics
        consecutive = consecutive + 1 if complete else 0
        if consecutive >= 2:
            return {
                "nodes": sorted(last_nodes),
                "topics": sorted(last_topics),
            }
        time.sleep(poll_s)
    missing_nodes = sorted(REQUIRED_READY_NODES - last_nodes)
    missing_topics = sorted(REQUIRED_READY_TOPICS - last_topics)
    raise QualificationHarnessError(
        f"runtime did not become ready; missing nodes={missing_nodes}, topics={missing_topics}"
    )


def validate_zero_step_artifacts(output_dir: Path) -> dict[str, object]:
    summary_path = output_dir / "summary.json"
    telemetry_path = output_dir / "controller_telemetry.csv"
    if not summary_path.is_file():
        raise QualificationHarnessError("zero-step summary.json was not produced")
    if not telemetry_path.is_file():
        raise QualificationHarnessError("zero-step controller telemetry was not produced")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    with telemetry_path.open(newline="", encoding="utf-8") as stream:
        telemetry = list(csv.DictReader(stream))

    required = {
        "simulator_step_count": 0,
        "accepted_drive_requests": 1,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "qualification_limit_reached": True,
        "fault": "qualification_step_limit_reached",
        "steps_after_terminal": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
    }
    for key, expected in required.items():
        if summary.get(key) != expected:
            raise QualificationHarnessError(
                f"zero-step evidence mismatch for {key}: {summary.get(key)!r} != {expected!r}"
            )
    if len(telemetry) != 1:
        raise QualificationHarnessError(
            f"expected one controller evaluation, observed {len(telemetry)}"
        )
    row = telemetry[0]
    steering = float(row["delta_applied_rad"])
    speed = float(row["upstream_linear_mps"])
    if not math.isfinite(steering) or not math.isfinite(speed):
        raise QualificationHarnessError("zero-step command is non-finite")
    if abs(steering) > 0.288 or speed < 0.0:
        raise QualificationHarnessError("zero-step command violates Ackermann limits")
    if row["physical_feasibility"].lower() not in {"1", "true"}:
        raise QualificationHarnessError("controller did not report physical feasibility")
    if row["safety_veto_pass"].lower() not in {"1", "true"}:
        raise QualificationHarnessError("independent safety veto rejected the command")
    return {
        "controller_evaluations": len(telemetry),
        "gym_steps": summary["simulator_step_count"],
        "state_stamp_ns": int(row["state_stamp_ns"]),
        "steering_rad": steering,
        "speed_mps": speed,
        "physical_feasibility": True,
        "safety_veto_pass": True,
    }


def validate_one_step_artifacts(output_dir: Path) -> dict[str, object]:
    """Prove exactly one state-command-step transition and no feedback of N+1."""

    summary_path = output_dir / "summary.json"
    telemetry_path = output_dir / "controller_telemetry.csv"
    trajectory_path = output_dir / "trajectory.csv"
    if not summary_path.is_file() or not telemetry_path.is_file() or not trajectory_path.is_file():
        raise QualificationHarnessError("one-step evidence artifacts are incomplete")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    with telemetry_path.open(newline="", encoding="utf-8") as stream:
        telemetry = list(csv.DictReader(stream))
    with trajectory_path.open(newline="", encoding="utf-8") as stream:
        trajectory = list(csv.DictReader(stream))

    required = {
        "simulator_step_count": 1,
        "accepted_drive_requests": 1,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "qualification_limit_reached": True,
        "fault": "qualification_step_limit_reached",
        "steps_after_terminal": 0,
        "collision_edges": 0,
        "off_track_events": 0,
        "reverse_command_events": 0,
        "invalid_command_events": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
    }
    for key, expected in required.items():
        if summary.get(key) != expected:
            raise QualificationHarnessError(
                f"one-step evidence mismatch for {key}: {summary.get(key)!r} != {expected!r}"
            )
    if len(telemetry) != 1:
        raise QualificationHarnessError(
            f"expected one controller evaluation, observed {len(telemetry)}"
        )
    if len(trajectory) != 1:
        raise QualificationHarnessError(
            f"expected one Gym state transition, observed {len(trajectory)}"
        )

    row = telemetry[0]
    transition = summary.get("qualification_transition")
    if not isinstance(transition, dict):
        raise QualificationHarnessError("one-step transition metadata is missing")
    state_n = transition.get("state_n")
    state_n1 = transition.get("state_n1")
    if not isinstance(state_n, dict) or not isinstance(state_n1, dict):
        raise QualificationHarnessError("one-step state metadata is missing")

    numeric_values = [
        float(row["upstream_linear_mps"]),
        float(row["upstream_angular_rps"]),
        float(row["kappa_cmd_1pm"]),
        float(row["delta_applied_rad"]),
        *[float(state_n[key]) for key in ("x_m", "y_m", "yaw_rad", "speed_mps")],
        *[float(state_n1[key]) for key in ("x_m", "y_m", "yaw_rad", "speed_mps")],
        float(state_n["full_body_clearance_m"]),
        float(state_n1["full_body_clearance_m"]),
    ]
    if not all(math.isfinite(value) for value in numeric_values):
        raise QualificationHarnessError("one-step evidence contains a non-finite value")

    state_stamp = int(state_n["stamp_ns"])
    state_n1_stamp = int(state_n1["stamp_ns"])
    if int(row["state_stamp_ns"]) != state_stamp:
        raise QualificationHarnessError("controller did not consume state N stamp")
    if state_n1_stamp - state_stamp != int(round(DT_S * 1_000_000_000)):
        raise QualificationHarnessError("state N+1 stamp does not advance by one simulator dt")
    if int(transition.get("command_index", -1)) != 1:
        raise QualificationHarnessError("Gym did not consume command index 1")

    steering = float(row["delta_applied_rad"])
    speed = float(row["upstream_linear_mps"])
    if abs(steering) > 0.288 or speed < 0.0 or speed > MAX_SPEED_MPS:
        raise QualificationHarnessError("one-step command violates Ackermann limits")
    if row["physical_feasibility"].lower() not in {"1", "true"}:
        raise QualificationHarnessError("one-step command is not physically feasible")
    if row["safety_veto_pass"].lower() not in {"1", "true"}:
        raise QualificationHarnessError("independent safety veto rejected the command")
    if row["downstream_steering_saturated"] not in {"0", "false", "False"}:
        raise QualificationHarnessError("downstream feasibility clamp activated")

    dx = float(state_n1["x_m"]) - float(state_n["x_m"])
    dy = float(state_n1["y_m"]) - float(state_n["y_m"])
    dyaw = float(state_n1["yaw_rad"]) - float(state_n["yaw_rad"])
    dspeed = float(state_n1["speed_mps"]) - float(state_n["speed_mps"])
    if math.hypot(dx, dy) > MAX_SPEED_MPS * DT_S * 1.1 + 1e-9:
        raise QualificationHarnessError("one-step state displacement is not physically bounded")
    if abs(steering) > 1e-12 and dyaw * steering <= 0.0:
        raise QualificationHarnessError("observed yaw direction opposes steering command")
    if bool(state_n1.get("collision")) or bool(state_n1.get("off_track")):
        raise QualificationHarnessError("one-step state is colliding or off track")

    return {
        "controller_evaluations": 1,
        "gym_steps": 1,
        "controller_input_stamp_ns": state_stamp,
        "controller_output_stamp_ns": state_stamp,
        "gym_input_command_index": 1,
        "state_n": state_n,
        "state_n1": state_n1,
        "vx_mps": speed,
        "wz_rps": float(row["upstream_angular_rps"]),
        "kappa_1pm": float(row["kappa_cmd_1pm"]),
        "steering_rad": steering,
        "downstream_feasibility_clamp_activations": 0,
        "safety_veto_pass": True,
        "dx_m": dx,
        "dy_m": dy,
        "dyaw_rad": dyaw,
        "dspeed_mps": dspeed,
        "steering_sign_semantics": "PASS",
    }


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def validate_short_horizon_artifacts(
    output_dir: Path,
    *,
    target_steps: int = SHORT_HORIZON_STEPS,
) -> dict[str, object]:
    """Validate and merge one complete deterministic multi-step lockstep run."""

    required_paths = {
        name: output_dir / name
        for name in (
            "summary.json", "controller_telemetry.csv", "controller_feasibility.csv",
            "trajectory.csv", "commands.csv",
        )
    }
    missing = [name for name, path in required_paths.items() if not path.is_file()]
    if missing:
        raise QualificationHarnessError(f"short-horizon evidence is incomplete: {missing}")
    summary = json.loads(required_paths["summary.json"].read_text(encoding="utf-8"))

    def read_csv(name: str) -> list[dict[str, str]]:
        with required_paths[name].open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))

    telemetry = read_csv("controller_telemetry.csv")
    feasibility = read_csv("controller_feasibility.csv")
    trajectory = read_csv("trajectory.csv")
    commands = read_csv("commands.csv")
    required_summary = {
        "simulator_step_count": target_steps,
        "accepted_drive_requests": target_steps,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "qualification_limit_reached": True,
        "fault": "qualification_step_limit_reached",
        "steps_after_terminal": 0,
        "collision_edges": 0,
        "off_track_events": 0,
        "reverse_command_events": 0,
        "invalid_command_events": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
    }
    for key, expected in required_summary.items():
        if summary.get(key) != expected:
            raise QualificationHarnessError(
                f"short-horizon evidence mismatch for {key}: "
                f"{summary.get(key)!r} != {expected!r}"
            )
    counts = {
        "controller telemetry": len(telemetry),
        "controller feasibility": len(feasibility),
        "trajectory": len(trajectory),
        "commands": len(commands),
    }
    for label, count in counts.items():
        if count != target_steps:
            raise QualificationHarnessError(
                f"expected {target_steps} {label} rows, observed {count}"
            )

    feasibility_by_stamp = {int(row["state_stamp_ns"]): row for row in feasibility}
    if len(feasibility_by_stamp) != target_steps:
        raise QualificationHarnessError("duplicate controller feasibility stamp")
    initial = summary.get("initial_state_evidence")
    if not isinstance(initial, dict):
        raise QualificationHarnessError("initial short-horizon state evidence is missing")

    merged: list[dict[str, object]] = []
    previous_stamp = 0
    previous_steering: float | None = None
    for expected_step, (controller, state, command) in enumerate(
        zip(telemetry, trajectory, commands), start=1
    ):
        if int(controller["sequence"]) != expected_step:
            raise QualificationHarnessError("controller sequence is not contiguous")
        if int(state["step"]) != expected_step or int(command["step"]) != expected_step:
            raise QualificationHarnessError("Gym/command sequence is not contiguous")
        stamp = int(controller["state_stamp_ns"])
        if stamp <= previous_stamp:
            raise QualificationHarnessError("controller state stamps are not strictly increasing")
        previous_stamp = stamp
        feasible = feasibility_by_stamp.get(stamp)
        if feasible is None:
            raise QualificationHarnessError("controller output has no matching feasibility record")
        input_stamp = int(feasible["controller_input_stamp_ns"])
        output_stamp = int(feasible["controller_output_stamp_ns"])
        if input_stamp != stamp or output_stamp != stamp:
            raise QualificationHarnessError("controller input/output stamp mismatch")

        vx = float(controller["upstream_linear_mps"])
        wz = float(controller["upstream_angular_rps"])
        kappa = float(controller["kappa_cmd_1pm"])
        steering = float(controller["delta_applied_rad"])
        controller_latency = float(feasible["controller_latency_ms"])
        gym_latency = float(state["gym_step_latency_ms"])
        actual_speed = float(state["actual_speed_mps"])
        clearance = float(state["full_body_clearance_m"])
        numeric = (
            vx, wz, kappa, steering, controller_latency, gym_latency, actual_speed,
            clearance, float(state["x_m"]), float(state["y_m"]),
            float(state["yaw_rad"]), float(state["signed_cte_m"]),
            float(state["heading_error_rad"]),
        )
        if not all(math.isfinite(value) for value in numeric):
            raise QualificationHarnessError(f"non-finite evidence at step {expected_step}")
        if abs(steering) > 0.288 or vx < 0.0 or vx > MAX_SPEED_MPS:
            raise QualificationHarnessError(f"invalid command at step {expected_step}")
        if controller["downstream_steering_saturated"] not in {"0", "false", "False"}:
            raise QualificationHarnessError(
                f"downstream feasibility clamp activated at step {expected_step}"
            )
        if feasible["physical_feasibility"].lower() not in {"1", "true"}:
            raise QualificationHarnessError(
                f"physical feasibility failed at step {expected_step}"
            )
        if feasible["safety_veto_pass"].lower() not in {"1", "true"}:
            raise QualificationHarnessError(f"safety veto at step {expected_step}")
        if int(state["collision"]) or int(state["off_track"]):
            raise QualificationHarnessError(
                f"collision/off-track state at step {expected_step}"
            )
        if not math.isclose(float(command["applied_speed_mps"]), vx, abs_tol=1e-12):
            raise QualificationHarnessError(f"executed speed mismatch at step {expected_step}")
        if not math.isclose(
            float(command["applied_steering_rad"]), steering, abs_tol=1e-8
        ):
            raise QualificationHarnessError(
                f"executed steering mismatch at step {expected_step}"
            )
        steering_delta = 0.0 if previous_steering is None else steering - previous_steering
        previous_steering = steering
        merged.append(
            {
                "step": expected_step,
                "sim_time_s": float(state["sim_time_s"]),
                "state_stamp_ns": stamp,
                "controller_input_stamp_ns": input_stamp,
                "controller_output_stamp_ns": output_stamp,
                "x_m": float(state["x_m"]),
                "y_m": float(state["y_m"]),
                "yaw_rad": float(state["yaw_rad"]),
                "actual_speed_mps": actual_speed,
                "command_vx_mps": vx,
                "command_wz_rps": wz,
                "curvature_1pm": kappa,
                "steering_rad": steering,
                "steering_delta_rad": steering_delta,
                "simulated_steering_rate_radps": steering_delta / DT_S,
                "signed_cte_m": float(state["signed_cte_m"]),
                "heading_error_rad": float(state["heading_error_rad"]),
                "full_body_clearance_m": clearance,
                "safety_veto_pass": True,
                "collision": 0,
                "off_track": 0,
                "reverse": int(vx < 0.0),
                "invalid_command": 0,
                "controller_latency_ms": controller_latency,
                "gym_step_latency_ms": gym_latency,
            }
        )

    merged_path = output_dir / "short_horizon_telemetry.csv"
    with merged_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(merged[0]))
        writer.writeheader()
        writer.writerows(merged)

    cte = [float(row["signed_cte_m"]) for row in merged]
    heading = [float(row["heading_error_rad"]) for row in merged]
    steering = [float(row["steering_rad"]) for row in merged]
    actual_speed = [float(row["actual_speed_mps"]) for row in merged]
    command_speed = [float(row["command_vx_mps"]) for row in merged]
    clearance = [float(row["full_body_clearance_m"]) for row in merged]
    controller_latency = [float(row["controller_latency_ms"]) for row in merged]
    gym_latency = [float(row["gym_step_latency_ms"]) for row in merged]
    steering_deltas = [abs(float(row["steering_delta_rad"])) for row in merged]
    initial_cte = float(initial["signed_cte_m"])
    initial_heading = float(initial["heading_error_rad"])
    min_clearance = min(clearance)
    min_clearance_step = clearance.index(min_clearance) + 1

    def state_at(step: int) -> dict[str, object]:
        return merged[step - 1]

    return {
        "target_steps": target_steps,
        "executed_steps": target_steps,
        "duration_s": float(summary["sim_time_s"]),
        "controller_evaluations": len(feasibility),
        "gym_steps": int(summary["simulator_step_count"]),
        "lockstep_causality": "PASS",
        "duplicate_stamp_count": 0,
        "stale_command_count": 0,
        "hidden_gym_step_count": 0,
        "cte_initial_m": initial_cte,
        "cte_final_m": cte[-1],
        "cte_rms_m": math.sqrt(sum(value * value for value in cte) / len(cte)),
        "cte_p95_m": _percentile([abs(value) for value in cte], 0.95),
        "cte_max_m": max(abs(value) for value in cte),
        "heading_initial_rad": initial_heading,
        "heading_final_rad": heading[-1],
        "heading_rms_rad": math.sqrt(
            sum(value * value for value in heading) / len(heading)
        ),
        "heading_p95_rad": _percentile([abs(value) for value in heading], 0.95),
        "heading_max_rad": max(abs(value) for value in heading),
        "s32": state_at(32),
        "s39": state_at(39),
        "max_abs_steering_rad": max(abs(value) for value in steering),
        "steering_saturation_count": sum(abs(value) >= 0.288 for value in steering),
        "steering_saturation_percent": (
            100.0 * sum(abs(value) >= 0.288 for value in steering) / len(steering)
        ),
        "max_command_delta_steering_rad": max(steering_deltas),
        "max_simulated_steering_rate_radps": max(value / DT_S for value in steering_deltas),
        "speed_initial_mps": float(initial["speed_mps"]),
        "speed_final_mps": actual_speed[-1],
        "speed_mean_mps": sum(actual_speed) / len(actual_speed),
        "speed_p95_mps": _percentile(actual_speed, 0.95),
        "speed_max_mps": max(actual_speed),
        "command_speed_mean_mps": sum(command_speed) / len(command_speed),
        "command_speed_max_mps": max(command_speed),
        "clearance_initial_m": float(initial["full_body_clearance_m"]),
        "clearance_final_m": clearance[-1],
        "min_full_body_clearance_m": min_clearance,
        "min_clearance_step": min_clearance_step,
        "collisions": 0,
        "off_track_events": 0,
        "reverse_commands": 0,
        "invalid_commands": 0,
        "safety_veto_count": 0,
        "downstream_feasibility_clamp_activations": 0,
        "controller_latency_p50_ms": _percentile(controller_latency, 0.50),
        "controller_latency_p95_ms": _percentile(controller_latency, 0.95),
        "controller_latency_max_ms": max(controller_latency),
        "gym_step_latency_p50_ms": _percentile(gym_latency, 0.50),
        "gym_step_latency_p95_ms": _percentile(gym_latency, 0.95),
        "gym_step_latency_max_ms": max(gym_latency),
        "telemetry_artifact": str(merged_path),
    }


def validate_trial1_artifacts(output_dir: Path) -> dict[str, object]:
    """Fail closed unless one full runtime satisfies the frozen ThreeLapGate."""

    required_paths = {
        name: output_dir / name
        for name in (
            "summary.json", "controller_telemetry.csv", "controller_feasibility.csv",
            "trajectory.csv", "commands.csv", "events.csv",
        )
    }
    missing = [name for name, path in required_paths.items() if not path.is_file()]
    if missing:
        raise QualificationHarnessError(f"Trial 1 evidence is incomplete: {missing}")

    summary = json.loads(required_paths["summary.json"].read_text(encoding="utf-8"))

    def read_csv(name: str) -> list[dict[str, str]]:
        with required_paths[name].open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))

    telemetry = read_csv("controller_telemetry.csv")
    feasibility = read_csv("controller_feasibility.csv")
    trajectory = read_csv("trajectory.csv")
    commands = read_csv("commands.csv")
    events = read_csv("events.csv")
    total_steps = int(summary.get("simulator_step_count", 0))
    required_summary = {
        "completed_laps": 3,
        "lap_count": 3,
        "done": True,
        "terminal_state": "COMPLETE",
        "fault": None,
        "steps_after_terminal": 0,
        "collision_edges": 0,
        "off_track_events": 0,
        "reverse_command_events": 0,
        "invalid_command_events": 0,
        "duplicate_state_stamp_rejections": 0,
        "state_stamp_mismatch_events": 0,
        "final_applied_command": {"steering_rad": 0.0, "speed_mps": 0.0},
        "terminal_zero_observed": True,
    }
    for key, expected in required_summary.items():
        if summary.get(key) != expected:
            raise QualificationHarnessError(
                f"Trial 1 evidence mismatch for {key}: {summary.get(key)!r} != {expected!r}"
            )
    if total_steps <= 0 or int(summary.get("accepted_drive_requests", 0)) != total_steps:
        raise QualificationHarnessError("Trial 1 has invalid step/request counts")
    acceptance = summary.get("acceptance")
    if not isinstance(acceptance, dict) or acceptance.get("overall") != "PASS":
        raise QualificationHarnessError("Trial 1 acceptance matrix did not pass")

    counts = {
        "controller telemetry": len(telemetry),
        "controller feasibility": len(feasibility),
        "trajectory": len(trajectory),
        "commands": len(commands),
    }
    for label, count in counts.items():
        if count != total_steps:
            raise QualificationHarnessError(
                f"expected {total_steps} {label} rows, observed {count}"
            )

    lap_events = [row["value"] for row in events if row["event"] == "lap_complete"]
    if lap_events != ["1", "2", "3"]:
        raise QualificationHarnessError(f"Trial 1 lap sequence is invalid: {lap_events}")
    terminal_events = [row["value"] for row in events if row["event"] == "terminal_state"]
    if terminal_events != ["COMPLETE"]:
        raise QualificationHarnessError(
            f"Trial 1 terminal event is invalid: {terminal_events}"
        )

    feasibility_by_stamp = {int(row["state_stamp_ns"]): row for row in feasibility}
    if len(feasibility_by_stamp) != total_steps:
        raise QualificationHarnessError("duplicate Trial 1 controller feasibility stamp")

    merged: list[dict[str, object]] = []
    previous_stamp = 0
    previous_steering: float | None = None
    previous_lap_count = 0
    for expected_step, (controller, state, command) in enumerate(
        zip(telemetry, trajectory, commands), start=1
    ):
        if int(controller["sequence"]) != expected_step:
            raise QualificationHarnessError("Trial 1 controller sequence is not contiguous")
        if int(state["step"]) != expected_step or int(command["step"]) != expected_step:
            raise QualificationHarnessError("Trial 1 Gym/command sequence is not contiguous")
        stamp = int(controller["state_stamp_ns"])
        if stamp <= previous_stamp:
            raise QualificationHarnessError("Trial 1 state stamps are not strictly increasing")
        previous_stamp = stamp
        feasible = feasibility_by_stamp.get(stamp)
        if feasible is None:
            raise QualificationHarnessError("Trial 1 command has no feasibility record")
        input_stamp = int(feasible["controller_input_stamp_ns"])
        output_stamp = int(feasible["controller_output_stamp_ns"])
        if input_stamp != stamp or output_stamp != stamp:
            raise QualificationHarnessError("Trial 1 controller stamp mismatch")

        vx = float(controller["upstream_linear_mps"])
        wz = float(controller["upstream_angular_rps"])
        kappa = float(controller["kappa_cmd_1pm"])
        steering = float(controller["delta_applied_rad"])
        values = (
            vx, wz, kappa, steering, float(feasible["controller_latency_ms"]),
            float(state["gym_step_latency_ms"]), float(state["actual_speed_mps"]),
            float(state["full_body_clearance_m"]), float(state["x_m"]),
            float(state["y_m"]), float(state["yaw_rad"]),
            float(state["signed_cte_m"]), float(state["heading_error_rad"]),
        )
        if not all(math.isfinite(value) for value in values):
            raise QualificationHarnessError(f"non-finite Trial 1 evidence at step {expected_step}")
        if vx < 0.0 or vx > MAX_SPEED_MPS or abs(steering) > 0.288:
            raise QualificationHarnessError(f"invalid Trial 1 command at step {expected_step}")
        if controller["downstream_steering_saturated"] not in {"0", "false", "False"}:
            raise QualificationHarnessError(
                f"Trial 1 feasibility clamp activated at step {expected_step}"
            )
        if feasible["physical_feasibility"].lower() not in {"1", "true"}:
            raise QualificationHarnessError(
                f"Trial 1 physical feasibility failed at step {expected_step}"
            )
        if feasible["safety_veto_pass"].lower() not in {"1", "true"}:
            raise QualificationHarnessError(f"Trial 1 safety veto at step {expected_step}")
        if int(state["collision"]) or int(state["off_track"]):
            raise QualificationHarnessError(
                f"Trial 1 collision/off-track at step {expected_step}"
            )
        if not math.isclose(float(command["applied_speed_mps"]), vx, abs_tol=1e-12):
            raise QualificationHarnessError(f"Trial 1 speed mismatch at step {expected_step}")
        if not math.isclose(
            float(command["applied_steering_rad"]), steering, abs_tol=1e-8
        ):
            raise QualificationHarnessError(f"Trial 1 steering mismatch at step {expected_step}")

        lap_count = int(state["lap_count"])
        if lap_count < previous_lap_count or lap_count > previous_lap_count + 1:
            raise QualificationHarnessError(f"Trial 1 lap jump at step {expected_step}")
        lap_number = previous_lap_count + 1
        if lap_count > previous_lap_count:
            previous_lap_count = lap_count
        steering_delta = 0.0 if previous_steering is None else steering - previous_steering
        previous_steering = steering
        merged.append(
            {
                "step": expected_step,
                "sim_time_s": float(state["sim_time_s"]),
                "lap_number": lap_number,
                "completed_laps": lap_count,
                "progress_index": int(controller["progress_index"]),
                "path_copy": int(controller["path_copy"]),
                "state_stamp_ns": stamp,
                "controller_input_stamp_ns": input_stamp,
                "controller_output_stamp_ns": output_stamp,
                "x_m": float(state["x_m"]),
                "y_m": float(state["y_m"]),
                "yaw_rad": float(state["yaw_rad"]),
                "actual_speed_mps": float(state["actual_speed_mps"]),
                "command_vx_mps": vx,
                "command_wz_rps": wz,
                "curvature_1pm": kappa,
                "steering_rad": steering,
                "steering_delta_rad": steering_delta,
                "simulated_steering_rate_radps": steering_delta / DT_S,
                "signed_cte_m": float(state["signed_cte_m"]),
                "heading_error_rad": float(state["heading_error_rad"]),
                "full_body_clearance_m": float(state["full_body_clearance_m"]),
                "safety_veto_pass": True,
                "collision": 0,
                "off_track": 0,
                "reverse": 0,
                "invalid_command": 0,
                "controller_latency_ms": float(feasible["controller_latency_ms"]),
                "gym_step_latency_ms": float(state["gym_step_latency_ms"]),
            }
        )
    if previous_lap_count != 3:
        raise QualificationHarnessError("Trial 1 merged telemetry did not reach lap 3")

    merged_path = output_dir / "trial1_telemetry.csv"
    with merged_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(merged[0]))
        writer.writeheader()
        writer.writerows(merged)
    return {
        "status": "PASS",
        "completed_laps": 3,
        "exact_three_laps": "PASS",
        "total_steps": total_steps,
        "duration_s": float(summary["sim_time_s"]),
        "controller_evaluations": len(feasibility),
        "gym_steps": total_steps,
        "lockstep_causality": "PASS",
        "duplicate_stamp_count": 0,
        "stale_command_count": 0,
        "hidden_gym_step_count": 0,
        "telemetry_artifact": str(merged_path),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=(
            "preflight", "readiness", "zero-step", "one-step", "short-horizon",
            "trial1",
        ),
        required=True,
    )
    parser.add_argument("--workspace", default="/tmp/laksa-c1.2-runtime")
    parser.add_argument(
        "--ackermann-prefix",
        default="/tmp/laksa-c1-native/ackermann_root/opt/ros/humble",
    )
    parser.add_argument(
        "--f1tenth-gym-checkout",
        default="/tmp/laksa-c1-native/f1tenth_gym",
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--domain-id")
    parser.add_argument("--domain-seed", type=int)
    parser.add_argument("--domain-lock-root", default="/tmp/laksa-c1-domain-locks")
    parser.add_argument("--timeout", type=float, default=120.0)
    return parser


def _launch_command(args: argparse.Namespace) -> list[str]:
    include_gym = "false" if args.mode == "readiness" else "true"
    gym_start_delay_s = "0.0" if args.mode == "readiness" else "30.0"
    if args.mode == "one-step":
        qualification_step_limit = "1"
    elif args.mode == "short-horizon":
        qualification_step_limit = str(SHORT_HORIZON_STEPS)
    elif args.mode == "trial1":
        qualification_step_limit = "-1"
    else:
        qualification_step_limit = "0"
    return [
        "ros2",
        "launch",
        "laksa_speed_race",
        "c1_nav2_mppi_three_lap.launch.py",
        f"output_dir:={args.output_dir}",
        f"qualification_step_limit:={qualification_step_limit}",
        f"include_gym:={include_gym}",
        f"gym_start_delay_s:={gym_start_delay_s}",
    ]


def main(argv: Iterable[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    evidence_path = output / "harness_result.json"
    if (output / "summary.json").exists() or (output / "controller_telemetry.csv").exists():
        raise SystemExit("refusing to reuse an output directory containing runtime evidence")

    evidence: dict[str, object] = {
        "mode": args.mode,
        "domain_id": None,
        "ready_state_criteria": {
            "nodes": sorted(REQUIRED_READY_NODES),
            "topics": sorted(REQUIRED_READY_TOPICS),
            "consecutive_snapshots": 2,
        },
        "status": "FAIL",
    }
    session: OwnedProcessSession | None = None
    domain_lease: RosDomainLease | None = None
    try:
        lock_root = Path(args.domain_lock_root)
        if args.domain_id is not None:
            domain_lease = claim_explicit_domain(
                args.domain_id,
                source="--domain-id",
                occupied=domain_has_active_graph,
                lock_root=lock_root,
            )
        else:
            seed = args.domain_seed
            if seed is None:
                digest = hashlib.sha256(str(output.resolve()).encode("utf-8")).digest()
                seed = int.from_bytes(digest[:8], "big", signed=False)
            domain_lease = allocate_isolated_domain(
                seed,
                occupied=domain_has_active_graph,
                lock_root=lock_root,
            )
        args.domain_id = domain_lease.domain_id
        evidence["domain_id"] = domain_lease.domain_id
        evidence["domain_source"] = domain_lease.source
        evidence["domain_lock_path"] = str(domain_lease.lock_path)
        preflight = validate_ackermann_runtime(args.ackermann_prefix)
        evidence["ackermann_preflight"] = preflight.to_dict()
        gym_preflight = validate_f1tenth_gym_runtime(
            args.f1tenth_gym_checkout,
            F1TENTH_GYM_SHA,
        )
        evidence["f1tenth_gym_preflight"] = gym_preflight.to_dict()
        if args.mode == "preflight":
            evidence["status"] = "PASS"
            return 0
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(args.domain_id)
        env["ROS_LOCALHOST_ONLY"] = "1"
        with (output / "launch.log").open("wb") as launch_log:
            session = OwnedProcessSession(
                _launch_command(args), env=env, stdout=launch_log, stderr=subprocess.STDOUT
            )
            if args.mode == "readiness":
                evidence["ready_graph"] = wait_for_ready(
                    session,
                    timeout_s=min(args.timeout, 30.0),
                    probe=lambda: runtime_graph_snapshot(env),
                )
                evidence["critical_children_ready"] = True
            else:
                evidence["ready_graph"] = wait_for_ready(
                    session,
                    timeout_s=min(args.timeout, 25.0),
                    probe=lambda: runtime_graph_snapshot(env),
                )
                evidence["critical_children_ready"] = True
                returncode = session.wait_for_leader(timeout=args.timeout)
                if returncode is None:
                    raise QualificationHarnessError(
                        f"{args.mode} launch did not terminate in time"
                    )
                evidence["launch_returncode"] = returncode
                if returncode != 0:
                    raise QualificationHarnessError(
                        f"{args.mode} launch exited with status {returncode}"
                    )
                if args.mode == "zero-step":
                    evidence["zero_step"] = validate_zero_step_artifacts(output)
                elif args.mode == "one-step":
                    evidence["one_step"] = validate_one_step_artifacts(output)
                elif args.mode == "short-horizon":
                    evidence["short_horizon"] = validate_short_horizon_artifacts(output)
                else:
                    evidence["trial1"] = validate_trial1_artifacts(output)

            shutdown = session.shutdown()
            evidence["shutdown"] = {
                "success": shutdown.success,
                "phase": shutdown.phase,
                "pgid": shutdown.pgid,
                "observed_pids": shutdown.observed_pids,
                "remaining_pids": shutdown.remaining_pids,
                "leader_returncode": shutdown.leader_returncode,
            }
            if not shutdown.success:
                raise QualificationHarnessError("owned runtime process group did not terminate")
        evidence["status"] = "PASS"
        return 0
    except (QualificationHarnessError, RosDomainError, RuntimePreflightError) as error:
        evidence["error"] = str(error)
        if session is not None:
            shutdown = session.shutdown()
            evidence["failure_shutdown"] = {
                "success": shutdown.success,
                "phase": shutdown.phase,
                "remaining_pids": shutdown.remaining_pids,
                "leader_returncode": shutdown.leader_returncode,
            }
        return 1
    finally:
        if domain_lease is not None:
            domain_lease.release()
        evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
