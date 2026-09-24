"""Persist and validate C1 run evidence.

This file performs evidence serialization only. It contains no navigation,
controller, optimization, or simulator mathematics.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

from .metrics import C1Metrics
from .three_lap_gate import MissionState, ThreeLapGate


def _write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def persist_run(
    output_dir: Path,
    *,
    metrics: C1Metrics,
    gate: ThreeLapGate,
    sim_time_s: float,
    metadata: dict[str, object],
    final_requested_command: dict[str, float],
    final_applied_command: dict[str, float],
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = metrics.summary(
        lap_times_s=gate.lap_times,
        sim_time_s=sim_time_s,
        done=gate.done,
        metadata=metadata,
        terminal_state=gate.state.value,
        final_requested_command=final_requested_command,
        final_applied_command=final_applied_command,
        steps_after_terminal=gate.steps_after_terminal,
    )
    summary["completed_laps"] = gate.lap_count
    summary["fault"] = gate.fault
    summary["terminal_zero_observed"] = gate.terminal_zero_published
    summary["acceptance"] = validate_summary(summary)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    _write_csv(
        output_dir / "trajectory.csv",
        metrics.trajectory_rows,
        [
            "step", "sim_time_s", "x_m", "y_m", "yaw_rad", "signed_cte_m",
            "heading_error_rad", "collision", "off_track", "lap_count",
        ],
    )
    _write_csv(
        output_dir / "commands.csv",
        metrics.command_rows,
        [
            "step", "sim_time_s", "requested_steering_rad", "requested_speed_mps",
            "applied_steering_rad", "applied_speed_mps", "mission_state",
        ],
    )
    events: list[dict[str, object]] = []
    previous_lap = 0
    collision_active = False
    off_track_active = False
    for row in metrics.trajectory_rows:
        lap_count = int(row["lap_count"])
        if lap_count > previous_lap:
            events.append(
                {
                    "step": row["step"],
                    "sim_time_s": row["sim_time_s"],
                    "event": "lap_complete",
                    "value": lap_count,
                }
            )
            previous_lap = lap_count
        collision = bool(row["collision"])
        if collision and not collision_active:
            events.append(
                {
                    "step": row["step"],
                    "sim_time_s": row["sim_time_s"],
                    "event": "collision_edge",
                    "value": 1,
                }
            )
        collision_active = collision
        off_track = bool(row["off_track"])
        if off_track and not off_track_active:
            events.append(
                {
                    "step": row["step"],
                    "sim_time_s": row["sim_time_s"],
                    "event": "off_track_edge",
                    "value": 1,
                }
            )
        off_track_active = off_track
    events.append(
        {
            "step": metrics.simulator_steps,
            "sim_time_s": sim_time_s,
            "event": "terminal_state",
            "value": gate.state.value if gate.fault is None else f"{gate.state.value}:{gate.fault}",
        }
    )
    _write_csv(
        output_dir / "events.csv",
        events,
        ["step", "sim_time_s", "event", "value"],
    )
    return summary


def validate_summary(summary: dict[str, object]) -> dict[str, str]:
    checks = {
        "exact_three_laps": "PASS" if summary.get("completed_laps") == 3 else "FAIL",
        "done": "PASS" if summary.get("done") is True else "FAIL",
        "terminal_state": "PASS" if summary.get("terminal_state") == MissionState.COMPLETE.value else "FAIL",
        "collision": "PASS" if summary.get("collision_edges") == 0 else "FAIL",
        "off_track": "PASS" if summary.get("off_track_events") == 0 else "FAIL",
        "reverse": "PASS" if summary.get("reverse_command_events") == 0 else "FAIL",
        "invalid_commands": "PASS" if summary.get("invalid_command_events") == 0 else "FAIL",
        "steering": "PASS" if float(summary.get("max_abs_steering_rad", 99.0)) <= 0.288 else "FAIL",
        "terminal_zero": "PASS"
        if summary.get("final_applied_command") == {"steering_rad": 0.0, "speed_mps": 0.0}
        else "FAIL",
        "steps_after_terminal": "PASS" if summary.get("steps_after_terminal") == 0 else "FAIL",
        "cte_threshold_unset": "PASS" if summary.get("cte_pass_threshold") == "UNSET" else "FAIL",
    }
    checks["overall"] = "PASS" if all(value == "PASS" for value in checks.values()) else "FAIL"
    return checks
