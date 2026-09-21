"""Independent path quality, oriented-footprint, and Reeds-Shepp metrics."""

from __future__ import annotations

import math
from statistics import fmean

from .geometry import reeds_shepp_kinematic_report, signed_segment_lengths, transformed_footprint, wrap_pi


def path_length(poses) -> float:
    return sum(abs(value) for value in signed_segment_lengths(poses))


def direction_changes(poses, epsilon: float = 1e-5) -> int:
    signs = [1 if value > 0 else -1 for value in signed_segment_lengths(poses, epsilon) if abs(value) > epsilon]
    return sum(first != second for first, second in zip(signs, signs[1:]))


def self_intersections(poses, epsilon: float = 1e-4) -> int:
    points = []
    for pose in poses:
        if not points or math.hypot(pose[0] - points[-1][0], pose[1] - points[-1][1]) > epsilon:
            points.append((pose[0], pose[1]))

    def orientation(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    count = 0
    for i in range(len(points) - 1):
        for j in range(i + 2, len(points) - 1):
            if i == 0 and j == len(points) - 2 and math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1]) <= epsilon:
                continue
            a, b, c, d = points[i], points[i + 1], points[j], points[j + 1]
            if orientation(a, b, c) * orientation(a, b, d) < 0 and orientation(c, d, a) * orientation(c, d, b) < 0:
                count += 1
    return count


def evaluate_path(poses, scenario: dict, map_data) -> dict:
    if len(poses) < 2 or any(not math.isfinite(value) for pose in poses for value in pose):
        return {"planning_success": False, "collision_free": False, "kinematically_feasible": False, "invalid_path": True}
    lengths = signed_segment_lengths(poses)
    kinematics = reeds_shepp_kinematic_report(poses)
    curvatures = [item["curvature_1pm"] for item in kinematics["curvature_samples"]]
    reverse_length = sum(abs(value) for value in lengths if value < 0)
    total_length = sum(abs(value) for value in lengths)
    clearance = [min(map_data.clearance_at(px, py) for px, py in ((pose[0], pose[1]), *transformed_footprint(*pose))) for pose in poses]
    goal = scenario["goal"]
    straight = math.hypot(goal["x"] - scenario["start"]["x"], goal["y"] - scenario["start"]["y"])
    goal_distances = [math.hypot(pose[0] - goal["x"], pose[1] - goal["y"]) for pose in poses]
    prefix = max(1, min(len(goal_distances), int(len(goal_distances) * 0.15)))
    move_away = max(0.0, max(goal_distances[:prefix]) - goal_distances[0])
    collision = map_data.path_collision_report(poses)
    result = {
        "planning_success": True, "invalid_path": False, "path_length_m": total_length,
        "minimum_clearance_m": min(clearance), "mean_clearance_m": fmean(clearance),
        "reverse_length_m": reverse_length, "reverse_fraction": reverse_length / total_length if total_length else 0.0,
        "direction_changes": kinematics["cusps"], "cusps": kinematics["cusps"],
        "self_intersections": self_intersections(poses),
        "max_abs_curvature": max((abs(value) for value in curvatures), default=0.0),
        "max_left_curvature": max((value for value in curvatures if value > 0), default=0.0),
        "max_right_curvature": min((value for value in curvatures if value < 0), default=0.0),
        "curvature_rms": math.sqrt(fmean([value * value for value in curvatures])) if curvatures else 0.0,
        "curvature_total_variation": kinematics["curvature_total_variation"],
        "endpoint_position_error": math.hypot(poses[-1][0] - goal["x"], poses[-1][1] - goal["y"]),
        "endpoint_yaw_error": abs(wrap_pi(poses[-1][2] - goal["yaw"])),
        "collision_free": collision["collision_free"],
        "collision_valid": collision["collision_free"],
        "collision_pose_index": collision.get("pose_index"),
        "collision_evidence": None if collision["collision_free"] else collision,
        "kinematically_feasible": kinematics["kinematically_feasible"],
        "kinematic_valid": kinematics["kinematically_feasible"],
        "move_away_first_m": move_away, "excess_path_ratio": total_length / straight if straight > 1e-6 else float("inf"),
    }
    result.update({key: value for key, value in kinematics.items() if key != "curvature_samples"})
    result["max_abs_curvature"] = result["max_abs_curvature_1pm"]
    return result
