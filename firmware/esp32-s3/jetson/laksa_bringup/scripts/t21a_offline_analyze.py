#!/usr/bin/env python3
"""Hash-bound, non-mutating analysis of completed supervised T21A sessions."""
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import tempfile
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mean(values):
    return statistics.fmean(values) if values else None


def analyze_session(path: Path):
    terminal_path = path / "analysis.json"; raw_path = path / "raw.jsonl"
    terminal = json.loads(terminal_path.read_text())
    rows = [json.loads(line) for line in raw_path.read_text().splitlines() if line.strip()]
    variant = terminal.get("variant")
    if terminal.get("test_id") != "T21A" or variant not in ("left", "right"):
        raise ValueError("T21A_SESSION_IDENTITY_INVALID")
    if terminal.get("status") != "COMPLETE" or not terminal.get("motion", {}).get("non_neutral_motion_requested"):
        raise ValueError("T21A_SESSION_NOT_COMPLETE_WITH_MOTION")
    cleanup = terminal.get("cleanup", {})
    if not cleanup.get("neutral_request_attempted") or not cleanup.get("gate_disable_attempted"):
        raise ValueError("T21A_SESSION_CLEANUP_NOT_PROVEN")
    hold = [row for row in rows if row.get("topic") == "/laksa/characterization_request" and
            abs(float(row.get("data", {}).get("steering_rad", 0.0))) > 0.02]
    if not hold:
        raise ValueError("T21A_STEER_HOLD_NOT_RECORDED")
    target = float(hold[0]["data"]["steering_rad"])
    if (variant == "left" and target <= 0) or (variant == "right" and target >= 0):
        raise ValueError("T21A_VARIANT_COMMAND_SIGN_MISMATCH")
    start, end = min(row["monotonic_ns"] for row in hold), max(row["monotonic_ns"] for row in hold)
    imu = [row for row in rows if row.get("topic") == "/laksa/imu/data"]
    base = [row["data"]["angular_velocity_rad_s"]["z"] for row in imu if start - 1_000_000_000 <= row["monotonic_ns"] < start]
    active = [row["data"]["angular_velocity_rad_s"]["z"] for row in imu if start <= row["monotonic_ns"] <= end]
    if len(base) < 3 or len(active) < 3:
        raise ValueError("T21A_IMU_SAMPLE_COVERAGE_INSUFFICIENT")
    baseline, held = mean(base), mean(active)
    return {"session": path.name, "variant": variant, "steering_target_rad": target,
            "steer_hold_duration_s": (end - start) / 1e9, "baseline_yaw_rate_z_rad_s": baseline,
            "hold_yaw_rate_z_rad_s": held, "yaw_rate_z_delta_rad_s": held - baseline,
            "peak_abs_yaw_rate_z_rad_s": max(active, key=abs), "baseline_sample_count": len(base),
            "hold_sample_count": len(active), "source_artifacts": {"raw_jsonl_sha256": sha(raw_path),
            "terminal_analysis_json_sha256": sha(terminal_path)}}


def report(left: Path, right: Path):
    trials = [analyze_session(left), analyze_session(right)]
    if {trial["variant"] for trial in trials} != {"left", "right"}:
        raise ValueError("T21A_LEFT_RIGHT_PAIR_REQUIRED")
    trials.sort(key=lambda trial: trial["variant"])
    left_trial = next(trial for trial in trials if trial["variant"] == "left")
    right_trial = next(trial for trial in trials if trial["variant"] == "right")
    opposed = left_trial["yaw_rate_z_delta_rad_s"] * right_trial["yaw_rate_z_delta_rad_s"] < 0
    return {"schema_version": "laksa-t21a-offline-analysis-v1", "test_id": "T21A",
            "status": "PASS" if opposed else "NEEDS_REVIEW", "classification": "LEFT_RIGHT_RESPONSE_OPPOSED" if opposed else "RIGHT_RESPONSE_NOT_OPPOSED",
            "trials": trials, "left_right_response_observed": opposed,
            "reason": None if opposed else "LEFT_RIGHT_YAW_SIGNS_NOT_OPPOSED",
            "raw_immutable": True, "non_motion_analysis": True,
            "limitations": ["Yaw is the IMU z channel validated by T21; no curvature, radius, or physical wheel angle is inferred."]}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", type=Path, required=True); parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(argv)
    value = report(args.left, args.right)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=args.output.parent, delete=False) as stream:
        json.dump(value, stream, indent=2, sort_keys=True); stream.write("\n"); temporary = Path(stream.name)
    temporary.replace(args.output)
    print(json.dumps(value, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
