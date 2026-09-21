"""Replay preserved planner cases through Nav2 and the independent validator.

This tool is deliberately planner-only: it requires ROS_DOMAIN_ID=71 and
ROS_LOCALHOST_ONLY=1, launches only map_server/planner_server/static TF, and
never creates a FollowPath, controller, or vehicle command client.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import traceback

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from .benchmark_runner import LabFailure, PlannerWorker, _path_tuples
from .geometry import signed_segment_lengths, stable_hash
from .map_dataset import discover_maps, load_map
from .metrics import evaluate_path


def _path_dict(path):
    return {
        "frame_id": path.header.frame_id,
        "poses": [
            {
                "frame_id": pose.header.frame_id,
                "stamp": {"sec": pose.header.stamp.sec, "nanosec": pose.header.stamp.nanosec},
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z,
                "qx": pose.pose.orientation.x,
                "qy": pose.pose.orientation.y,
                "qz": pose.pose.orientation.z,
                "qw": pose.pose.orientation.w,
            }
            for pose in path.poses
        ],
    }


def _distance_at(poses, index, fraction=0.0):
    distances = [math.hypot(second[0] - first[0], second[1] - first[1]) for first, second in zip(poses, poses[1:])]
    return sum(distances[:max(0, index)]) + (distances[index] * fraction if 0 <= index < len(distances) else 0.0)


def _direction_report(poses):
    lengths = signed_segment_lengths(poses)
    report, last_direction = [], None
    for index, length in enumerate(lengths):
        if abs(length) < 1.0e-6:
            continue
        direction = "FORWARD" if length > 0.0 else "REVERSE"
        if direction != last_direction:
            report.append({"start_pose_index": index, "distance_along_path_m": _distance_at(poses, index), "direction": direction})
            last_direction = direction
    return {"segments": report, "reverse_segment_count": sum(item["direction"] == "REVERSE" for item in report),
            "cusp_positions_m": [item["distance_along_path_m"] for item in report[1:]]}


def _first_failure(poses, independent):
    collision = independent.get("collision_evidence") or {}
    if collision:
        index, fraction = collision.get("pose_index"), collision.get("sample_fraction", 0.0)
        return {
            "kind": "COLLISION",
            "path_index": index,
            "sample_fraction": fraction,
            "distance_along_path_m": _distance_at(poses, index, fraction),
            "xy_yaw": {key: collision.get(key) for key in ("x", "y", "yaw")},
            "reason": collision.get("reason"),
            "offending_cells": collision.get("offending_cells", []),
        }
    index = independent.get("kinematic_violation_pose_index")
    if index is not None:
        return {
            "kind": "KINEMATIC",
            "path_index": index,
            "distance_along_path_m": _distance_at(poses, index),
            "curvature_1pm": independent.get("max_abs_curvature_1pm"),
            "minimum_turning_radius_m": independent.get("min_implied_radius_m"),
            "heading_delta_rad": independent.get("heading_delta_rad"),
            "segment_length_m": independent.get("segment_length_m"),
            "signed_direction": independent.get("signed_direction"),
        }
    return None


def _classification(official, independent):
    if official["is_valid"] and independent["collision_free"] and independent["kinematically_feasible"]:
        return "OTHER:VALID"
    if official["is_valid"] and not independent["kinematically_feasible"]:
        return "KINEMATIC_MODEL_MISMATCH"
    if official["is_valid"] and not independent["collision_free"]:
        return "FOOTPRINT_MISMATCH"
    if not official["is_valid"] and independent["collision_free"] and independent["kinematically_feasible"]:
        return "NAV2_VALIDATOR_CONFIGURATION_MISMATCH"
    return "PLANNER_ACTUAL_INVALID_PATH"


class _Context:
    def __init__(self, results):
        self.results = results
        self.share = Path(get_package_share_directory("laksa_planning_lab"))
        self.lattices = {}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Planner-only official Nav2 path-validity forensics")
    parser.add_argument("--scenarios", type=Path, required=True)
    parser.add_argument("--sessions-root", type=Path, default=Path("/home/ubuntu/laksa_mapping_sessions"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if os.environ.get("ROS_DOMAIN_ID") != "71" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Safety gate: set ROS_DOMAIN_ID=71 and ROS_LOCALHOST_ONLY=1")
    dataset = json.loads(args.scenarios.read_text(encoding="utf-8"))
    scenarios = dataset["scenarios"]
    entries = {entry["map_id"]: entry for entry in discover_maps(args.sessions_root)}
    missing = [item["map_id"] for item in scenarios if item["map_id"] not in entries]
    if missing:
        raise SystemExit(f"Preserved map(s) unavailable: {sorted(set(missing))}")
    for item in scenarios:
        if entries[item["map_id"]]["sha256"] != item["map_sha256"]:
            raise SystemExit(f"Map hash drift for {item['scenario_id']}: {item['map_id']}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    context = _Context(args.output.parent)
    configuration = {"method": "HYBRID_PRODUCTION"}
    digest = stable_hash({"forensics": "official-is-path-valid-v1", "scenarios": scenarios})
    maps = {map_id: load_map(Path(entry["yaml"]), map_id) for map_id, entry in entries.items()}
    runtime = args.output.parent / "runtime" / f"validity_forensics_{digest[:12]}.yaml"
    # PlannerWorker renders the immutable production REEDS_SHEPP config.
    worker, cases = None, []
    rclpy.init(args=[])
    try:
        first = scenarios[0]
        worker = PlannerWorker(context, "HYBRID_PRODUCTION", configuration, digest, entries[first["map_id"]], maps[first["map_id"]])
        current_map = first["map_id"]
        for scenario in scenarios:
            map_id = scenario["map_id"]
            if map_id != current_map:
                worker.switch_map(entries[map_id], maps[map_id])
                current_map = map_id
            poses, planning_ms, smoothing_ms, total_ms = worker.client.plan(scenario)
            path = copy.deepcopy(worker.client.last_path)
            official = worker.client.validate_path(path)
            independent = evaluate_path(poses, scenario, maps[map_id])
            cases.append({
                "case": scenario["scenario_id"], "map": entries[map_id], "start": scenario["start"], "goal": scenario["goal"],
                "compute_path_to_pose": {"success": True, "planning_time_ms": planning_ms, "smoothing_time_ms": smoothing_ms, "total_pipeline_time_ms": total_ms},
                "returned_path": _path_dict(path), "returned_path_sha256": stable_hash(_path_dict(path)),
                "nav2_is_path_valid": official, "independent_validator": independent,
                "agreement": official["is_valid"] == bool(independent["collision_free"] and independent["kinematically_feasible"]),
                "primary_classification": _classification(official, independent),
                "motion": _direction_report(poses), "first_independent_failure": _first_failure(poses, independent),
            })
        payload = {
            "schema_version": 1,
            "safety": {"ros_domain_id": os.environ["ROS_DOMAIN_ID"], "ros_localhost_only": os.environ["ROS_LOCALHOST_ONLY"], "follow_path_created": False, "controller_started": False},
            "official_validator": {"service": "/laksa_planning_lab/is_path_valid", "interface": "nav2_msgs/srv/IsPathValid", "owner": "nav2_planner/planner_server"},
            "planner": {"plugin": "nav2_smac_planner/SmacPlannerHybrid", "motion_model": "REEDS_SHEPP", "runtime_params_file": str(runtime)},
            "scenarios_source": str(args.scenarios.resolve()), "scenarios_sha256": stable_hash(scenarios),
            "runtime_costmap": worker.client.costmap_snapshot, "runtime_footprint": worker.client.published_footprint,
            "cases": cases,
        }
        args.output.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        print(f"wrote {len(cases)} cases to {args.output}")
    finally:
        if worker is not None:
            worker.close()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise
