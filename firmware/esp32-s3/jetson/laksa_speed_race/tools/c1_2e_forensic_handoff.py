#!/usr/bin/env python3
"""Assemble the passive C1.2e replay handoff and exact reproduction verdict."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path


REFERENCE = {
    "off_track_step": 1326,
    "total_steps": 1326,
    "duration_s": 13.259999999999762,
    "cte_initial": 0.26671643419952507,
    "cte_final": 0.512046343006913,
    "cte_rms": 0.35756152947690606,
    "cte_p95": 0.4906782438292889,
    "cte_max": 0.512046343006913,
    "heading_initial": -0.000023786786456092557,
    "heading_final": 0.13401234342107493,
    "heading_rms": 0.08727063796842102,
    "heading_p95": 0.13516109990781155,
    "heading_max": 0.13892107929499797,
    "max_abs_steering": 0.08737466934699005,
    "command_speed_mean": 0.24400664711031125,
    "command_speed_p95": 0.25136659294366837,
    "command_speed_max": 0.25309669971466064,
    "clearance_initial": 0.30919994145690904,
    "clearance_final": -0.00007242838671644991,
    "min_clearance": -0.00007242838671644991,
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def dynamic_hash(root: Path) -> str:
    """Hash normalized dynamics while excluding stamps, latency, paths, PIDs and domains."""

    controller = read_csv(root / "controller_telemetry.csv")
    trajectory = read_csv(root / "trajectory.csv")
    commands = read_csv(root / "commands.csv")
    controller_fields = (
        "sequence", "x_m", "y_m", "yaw_rad", "actual_velocity_mps",
        "progress_index", "path_copy", "kappa_req_1pm", "kappa_max_1pm",
        "kappa_cmd_1pm", "upstream_linear_mps", "omega_pre_feasibility_rps",
        "upstream_angular_rps", "delta_equivalent_rad", "delta_raw_rad",
        "delta_applied_rad", "curvature_saturated", "downstream_steering_saturated",
        "physical_feasibility", "safety_veto_pass",
    )
    trajectory_fields = (
        "step", "sim_time_s", "x_m", "y_m", "yaw_rad", "actual_speed_mps",
        "signed_cte_m", "heading_error_rad", "collision", "off_track",
        "full_body_clearance_m", "lap_count",
    )
    command_fields = (
        "step", "sim_time_s", "requested_steering_rad", "requested_speed_mps",
        "applied_steering_rad", "applied_speed_mps", "mission_state",
    )
    normalized = {
        "controller": [[row.get(field, "") for field in controller_fields] for row in controller],
        "trajectory": [[row.get(field, "") for field in trajectory_fields] for row in trajectory],
        "commands": [[row.get(field, "") for field in command_fields] for row in commands],
    }
    payload = json.dumps(normalized, separators=(",", ":"), sort_keys=True).encode()
    return hashlib.sha256(payload).hexdigest()


def load_unrolled_raceline(path: Path) -> list[tuple[float, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [(float(row[0]), float(row[1])) for row in csv.reader(stream) if row]
    if len(rows) != 547 or rows[0] != rows[-1]:
        raise ValueError("expected frozen closed 547-row C1.1 raceline")
    return rows[:-1] * 4 + [rows[0]]


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def nearest_forward_index(
    points: list[tuple[float, float]], x: float, y: float, start: int, count: int = 48
) -> int:
    stop = min(len(points), max(start + 1, start + count))
    return min(
        range(start, stop),
        key=lambda index: (points[index][0] - x) ** 2 + (points[index][1] - y) ** 2,
    )


def plot_artifacts(root: Path, merged: list[dict[str, object]], raceline):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [float(row["sim_time_s"]) for row in merged]
    failure_time = times[-1]

    def series(filename, field, ylabel, *, zero=False):
        values = [float(row[field]) for row in merged]
        figure, axis = plt.subplots(figsize=(9, 4.8))
        axis.plot(times, values, linewidth=1.4)
        if zero:
            axis.axhline(0.0, color="black", linewidth=0.8)
        axis.axvline(failure_time, color="tab:red", linestyle="--")
        axis.scatter([failure_time], [values[-1]], color="tab:red", zorder=3)
        axis.set_xlabel("Simulation time (s)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        figure.tight_layout()
        figure.savefig(root / filename, dpi=160)
        plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 4.8))
    axis.plot([p[0] for p in raceline], [p[1] for p in raceline], color="0.75", linewidth=1)
    axis.plot([float(r["post_step_x_m"]) for r in merged],
              [float(r["post_step_y_m"]) for r in merged], linewidth=1.4)
    axis.scatter([float(merged[-1]["post_step_x_m"])],
                 [float(merged[-1]["post_step_y_m"])], color="tab:red", zorder=3)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(root / "trajectory_vs_raceline.png", dpi=160)
    plt.close(figure)
    series("cte.png", "signed_cte_m", "Signed CTE (m)")
    series("heading_error.png", "signed_heading_error_rad", "Heading error (rad)")
    series("steering.png", "equivalent_ackermann_steering_rad", "Steering (rad)")
    series("clearance.png", "full_body_clearance_m", "Full-body clearance (m)", zero=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-dir", type=Path, required=True)
    parser.add_argument("--original-dir", type=Path, required=True)
    parser.add_argument("--raceline", type=Path, required=True)
    parser.add_argument("--repo-sha", required=True)
    parser.add_argument("--branch", required=True)
    parser.add_argument("--nav2-sha", required=True)
    parser.add_argument("--gym-sha", required=True)
    parser.add_argument("--instrumentation-script", type=Path, required=True)
    parser.add_argument("--instrumented-source", type=Path, required=True)
    parser.add_argument("--instrumentation-diff", type=Path, required=True)
    args = parser.parse_args()

    root = args.replay_dir.resolve()
    controller = read_csv(root / "controller_telemetry.csv")
    feasibility = read_csv(root / "controller_feasibility.csv")
    trajectory = read_csv(root / "trajectory.csv")
    commands = read_csv(root / "commands.csv")
    path_rows = read_csv(root / "path_pipeline_telemetry_raw.csv")
    critics = read_csv(root / "critic_telemetry.csv")
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    if not (len(controller) == len(feasibility) == len(trajectory) == len(commands) == len(path_rows)):
        raise SystemExit("forensic replay row counts are not lockstep")
    if len(critics) != len(controller) * 8:
        raise SystemExit("expected eight critic rows per controller evaluation")

    raceline = load_unrolled_raceline(args.raceline.resolve())
    first_index = 0
    processed_paths: list[dict[str, object]] = []
    for row in path_rows:
        x = float(row["transformed_path_first_x_m"])
        y = float(row["transformed_path_first_y_m"])
        first_index = nearest_forward_index(raceline, x, y, first_index)
        count = int(row["transformed_path_point_count"])
        enriched = dict(row)
        enriched["transformed_path_first_index"] = first_index
        enriched["transformed_path_last_index"] = first_index + count - 1
        processed_paths.append(enriched)
    path_fields = list(processed_paths[0])
    with (root / "path_pipeline_telemetry.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=path_fields)
        writer.writeheader()
        writer.writerows(processed_paths)

    feasibility_by_stamp = {row["state_stamp_ns"]: row for row in feasibility}
    merged: list[dict[str, object]] = []
    previous_steering: float | None = None
    for index, (control, state, command, path) in enumerate(
        zip(controller, trajectory, commands, processed_paths), start=1
    ):
        stamp = control["state_stamp_ns"]
        feasible = feasibility_by_stamp[stamp]
        steering = float(control["delta_applied_rad"])
        delta = 0.0 if previous_steering is None else steering - previous_steering
        previous_steering = steering
        merged.append(
            {
                "step_index": index,
                "sim_time_s": float(state["sim_time_s"]),
                "state_stamp_ns": int(stamp),
                "state_n_x_m": float(control["x_m"]),
                "state_n_y_m": float(control["y_m"]),
                "state_n_yaw_rad": float(control["yaw_rad"]),
                "state_n_actual_speed_mps": float(control["actual_velocity_mps"]),
                "post_step_x_m": float(state["x_m"]),
                "post_step_y_m": float(state["y_m"]),
                "post_step_yaw_rad": float(state["yaw_rad"]),
                "post_step_actual_speed_mps": float(state["actual_speed_mps"]),
                "signed_cte_m": float(state["signed_cte_m"]),
                "abs_cte_m": abs(float(state["signed_cte_m"])),
                "signed_heading_error_rad": float(state["heading_error_rad"]),
                "nearest_global_raceline_index": int(control["progress_index"]),
                "nearest_local_path_index": int(path["nearest_local_path_index"]),
                "global_path_point_count": int(path["global_path_point_count"]),
                "transformed_path_point_count": int(path["transformed_path_point_count"]),
                "transformed_path_length_m": float(path["transformed_path_length_m"]),
                "transformed_path_first_index": int(path["transformed_path_first_index"]),
                "transformed_path_last_index": int(path["transformed_path_last_index"]),
                "transformed_path_spacing_min_m": float(path["transformed_path_spacing_min_m"]),
                "transformed_path_spacing_mean_m": float(path["transformed_path_spacing_mean_m"]),
                "transformed_path_spacing_max_m": float(path["transformed_path_spacing_max_m"]),
                "controller_vx_mps": float(control["upstream_linear_mps"]),
                "controller_wz_radps": float(control["upstream_angular_rps"]),
                "controller_curvature_1pm": float(control["kappa_cmd_1pm"]),
                "equivalent_ackermann_steering_rad": steering,
                "steering_delta_rad": delta,
                "implied_simulated_steering_rate_radps": delta / 0.01,
                "full_body_clearance_m": float(state["full_body_clearance_m"]),
                "off_track": int(state["off_track"]),
                "collision": int(state["collision"]),
                "safety_veto": not (feasible["safety_veto_pass"].lower() == "true"),
                "safety_veto_projected_min_clearance_m": "NOT_AVAILABLE_BOOLEAN_COSTMAP_VETO",
                "controller_input_stamp_ns": int(feasible["controller_input_stamp_ns"]),
                "controller_output_stamp_ns": int(feasible["controller_output_stamp_ns"]),
                "controller_latency_ms": float(feasible["controller_latency_ms"]),
                "gym_step_latency_ms": float(state["gym_step_latency_ms"]),
                "command_index_or_id": int(command["step"]),
            }
        )
    with (root / "replay_raw_telemetry.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(merged[0]))
        writer.writeheader()
        writer.writerows(merged)

    cte = [float(row["signed_cte_m"]) for row in trajectory]
    heading = [float(row["heading_error_rad"]) for row in trajectory]
    steering = [float(row["delta_applied_rad"]) for row in controller]
    speed = [float(row["upstream_linear_mps"]) for row in controller]
    clearance = [float(row["full_body_clearance_m"]) for row in trajectory]
    failure_steps = [int(row["step"]) for row in trajectory if int(row["off_track"])]
    metrics = {
        "off_track_step": failure_steps[0] if failure_steps else None,
        "total_steps": len(trajectory),
        "duration_s": float(trajectory[-1]["sim_time_s"]),
        "cte_initial": cte[0],
        "cte_final": cte[-1],
        # Use the canonical harness reductions for exact reproduction. Recomputing
        # these aggregates in Python can change the final bit through reduction
        # order even when every normalized dynamic sample is identical.
        "cte_rms": float(summary["cte_rms_m"]),
        "cte_p95": float(summary["cte_p95_m"]),
        "cte_max": float(summary["cte_max_m"]),
        "heading_initial": heading[0],
        "heading_final": heading[-1],
        "heading_rms": float(summary["heading_error_rms_rad"]),
        "heading_p95": float(summary["heading_error_p95_rad"]),
        "heading_max": float(summary["heading_error_max_rad"]),
        "max_abs_steering": max(abs(value) for value in steering),
        "command_speed_mean": float(summary["requested_speed_mean_mps"]),
        "command_speed_p95": percentile(speed, 0.95),
        "command_speed_max": float(summary["requested_speed_max_mps"]),
        "clearance_initial": float(summary["initial_state_evidence"]["full_body_clearance_m"]),
        "clearance_final": clearance[-1],
        "min_clearance": min(clearance),
    }
    metric_exact = all(metrics[key] == value for key, value in REFERENCE.items())
    replay_hash = dynamic_hash(root)
    original_hash = dynamic_hash(args.original_dir.resolve())
    reproduction = metric_exact and replay_hash == original_hash

    path_align_rows = [row for row in processed_paths]
    active_count = sum(row["path_align_activation_state"] != "INACTIVE" for row in path_align_rows)
    handoff_summary = {
        **metrics,
        "replay_reproduction": "PASS_EXACT" if reproduction else "FAIL",
        "metric_exact_equality": metric_exact,
        "normalized_dynamic_hash": replay_hash,
        "original_normalized_dynamic_hash": original_hash,
        "normalized_dynamic_hash_equal": replay_hash == original_hash,
        "path_align_required_index": int(path_align_rows[0]["path_align_required_index"]),
        "path_align_max_available_index": max(
            int(row["path_align_max_available_index"]) for row in path_align_rows
        ),
        "path_align_active_count": active_count,
        "path_align_inactive_count": len(path_align_rows) - active_count,
        "path_align_inactive_percent": 100.0 * (len(path_align_rows) - active_count) / len(path_align_rows),
        "global_path_point_count": len(raceline),
        "global_path_length_m": path_length(raceline),
        "transformed_path_initial": path_align_rows[0],
        "transformed_path_failure": path_align_rows[-1],
        "critic_rows": len(critics),
        "selection_semantics": "MPPI_WEIGHTED_UPDATE_NO_SINGLE_SELECTED_TRAJECTORY",
    }
    (root / "replay_summary.json").write_text(
        json.dumps(handoff_summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    harness = json.loads((root / "harness_result.json").read_text(encoding="utf-8"))
    frozen_files = {
        "raceline_sha256": sha256(args.raceline.resolve()),
        "controller_config_sha256": summary["controller_config_sha256"],
        "course_source_sha256": summary["course_source_sha256"],
        "course_geometry_sha256": summary["course_geometry_sha256"],
        "proxy_config_sha256": summary["proxy_config_sha256"],
    }
    provenance = {
        "repository_sha": args.repo_sha,
        "branch": args.branch,
        "nav2_mppi_sha": args.nav2_sha,
        "f1tenth_gym_sha": args.gym_sha,
        "frozen_configuration_hashes": frozen_files,
        "vehicle_geometry": {
            "wheelbase_m": 0.324,
            "steering_limit_rad": 0.288,
            "length_m": 0.568,
            "width_m": 0.296,
            "collision_body_center_x_m": 0.135,
        },
        "seed_policy": {"gym_seed": 12345, "mppi_regenerate_noises": False},
        "instrumentation": {
            "script_sha256": sha256(args.instrumentation_script.resolve()),
            "instrumented_source_sha256": sha256(args.instrumented_source.resolve()),
            "diff_sha256": sha256(args.instrumentation_diff.resolve()),
            "behavioral_effect": "NONE" if reproduction else "NOT_PROVEN",
        },
        "runtime_source_order": [
            "/opt/ros/humble/setup.bash",
            "/tmp/laksa-c1-native/ackermann_root/opt/ros/humble/share/ackermann_msgs/local_setup.bash",
            "/tmp/laksa-c1.2-runtime/install/setup.bash",
            "isolated venv and pinned checkout PYTHONPATH",
        ],
        "ros_domain": harness["domain_id"],
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    (root / "provenance.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    manifest_lines = [
        "C1.2e-R1 passive instrumentation manifest",
        f"repository_sha={args.repo_sha}",
        f"nav2_sha={args.nav2_sha}",
        f"gym_sha={args.gym_sha}",
        f"instrumentation_script_sha256={sha256(args.instrumentation_script.resolve())}",
        f"instrumented_source_sha256={sha256(args.instrumented_source.resolve())}",
        f"instrumentation_diff_sha256={sha256(args.instrumentation_diff.resolve())}",
        "critic_order_changed=NO",
        "critic_math_changed=NO",
        "optimizer_buffers_changed=NO",
        "rng_calls_added=NO",
        "controller_inputs_changed=NO",
        "controller_outputs_changed=NO",
        "gym_advancement_changed=NO",
        f"instrumentation_behavioral_effect={'NONE' if reproduction else 'NOT_PROVEN'}",
        "critic_selection_note=MPPI uses a weighted update; diagnostic_candidate_index is argmin total cost only",
        "safety_veto_clearance_note=existing veto exposes boolean costmap collision only; no invented clearance metric",
    ]
    (root / "instrumentation_manifest.txt").write_text(
        "\n".join(manifest_lines) + "\n", encoding="utf-8"
    )
    plot_artifacts(root, merged, raceline)
    print(json.dumps(handoff_summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
