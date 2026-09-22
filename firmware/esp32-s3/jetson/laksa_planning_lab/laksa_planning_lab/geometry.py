"""Measured LAKSA geometry and deterministic planar math."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Iterable, Sequence

WHEELBASE_M = 0.324
LEFT_STEERING_LIMIT_RAD = 0.523
RIGHT_STEERING_LIMIT_RAD = -0.288
FOOTPRINT = ((0.419, 0.148), (0.419, -0.148), (-0.149, -0.148), (-0.149, 0.148))
CONSERVATIVE_RADIUS_M = 1.09
KINEMATIC_RELATIVE_TOLERANCE = 0.05
MIN_KINEMATIC_SEGMENT_M = 1.0e-3


def turning_radius(steering_rad: float) -> float:
    return WHEELBASE_M / math.tan(abs(steering_rad))


R_LEFT_M = turning_radius(LEFT_STEERING_LIMIT_RAD)
R_RIGHT_M = turning_radius(RIGHT_STEERING_LIMIT_RAD)
KAPPA_LEFT_MAX = 1.0 / R_LEFT_M
KAPPA_RIGHT_ABS_MAX = 1.0 / R_RIGHT_M


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def auto_heading(start_x: float, start_y: float, start_yaw: float, goal_x: float, goal_y: float) -> float:
    dx, dy = goal_x - start_x, goal_y - start_y
    distance = math.hypot(dx, dy)
    desired_delta = wrap_pi(math.atan2(dy, dx) - start_yaw)
    limit = min(math.pi, distance / CONSERVATIVE_RADIUS_M)
    return wrap_pi(start_yaw + max(-limit, min(limit, desired_delta)))


def stable_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def signed_segment_lengths(poses: Sequence[Sequence[float]], epsilon: float = 1e-6) -> list[float]:
    result: list[float] = []
    last_sign = 1.0
    for first, second in zip(poses, poses[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        if length <= epsilon:
            result.append(0.0)
            continue
        dot = dx * math.cos(first[2]) + dy * math.sin(first[2])
        if abs(dot) > epsilon:
            last_sign = 1.0 if dot > 0.0 else -1.0
        result.append(last_sign * length)
    return result


def signed_curvatures(poses: Sequence[Sequence[float]], epsilon: float = 1e-6) -> list[float]:
    distances = signed_segment_lengths(poses, epsilon)
    return [wrap_pi(b[2] - a[2]) / distance for a, b, distance in zip(poses, poses[1:], distances) if abs(distance) > epsilon]


def reeds_shepp_kinematic_report(
    poses: Sequence[Sequence[float]],
    minimum_turning_radius: float = CONSERVATIVE_RADIUS_M,
    relative_tolerance: float = KINEMATIC_RELATIVE_TOLERANCE,
) -> dict:
    """Validate spatial curvature within same-direction Reeds-Shepp branches.

    A 5% tolerance covers finite map/lattice discretization only. Degenerate
    samples and the vertex at a forward/reverse cusp are never differentiated.
    """
    clean = []
    for original_index, pose in enumerate(poses):
        point = (float(pose[0]), float(pose[1]), float(pose[2]))
        if clean and math.hypot(point[0] - clean[-1][1][0], point[1] - clean[-1][1][1]) < MIN_KINEMATIC_SEGMENT_M:
            continue
        clean.append((original_index, point))
    segments = []
    cumulative = [0.0]
    for (_, first), (_, second) in zip(clean, clean[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        midpoint_heading = first[2] + 0.5 * wrap_pi(second[2] - first[2])
        direction = 1 if math.cos(wrap_pi(bearing - midpoint_heading)) >= 0.0 else -1
        segments.append({"length": length, "bearing": bearing, "direction": direction})
        cumulative.append(cumulative[-1] + length)
    cusp_vertices = [
        index for index in range(1, len(clean) - 1)
        if segments[index - 1]["direction"] != segments[index]["direction"]
    ]
    cusp_distances = [cumulative[index] for index in cusp_vertices]
    candidates = []
    group = 0
    for index in range(1, len(clean) - 1):
        before, after = segments[index - 1], segments[index]
        if before["direction"] != after["direction"]:
            group += 1
            continue
        p0, p1, p2 = clean[index - 1][1], clean[index][1], clean[index + 1][1]
        chord = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
        denominator = before["length"] * after["length"] * chord
        if denominator <= MIN_KINEMATIC_SEGMENT_M ** 3:
            continue
        cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])
        geometric_curvature = 2.0 * cross / denominator
        signed_curvature = geometric_curvature * before["direction"]
        nearest_cusp = min((abs(cumulative[index] - value) for value in cusp_distances), default=None)
        candidates.append({
            "pose_index": clean[index][0],
            "curvature_1pm": signed_curvature,
            "segment_length_m": 0.5 * (before["length"] + after["length"]),
            "heading_delta_rad": wrap_pi(after["bearing"] - before["bearing"]),
            "signed_direction": before["direction"],
            "nearest_cusp_distance_m": nearest_cusp,
            "group": group,
        })
    maximum = max((abs(item["curvature_1pm"]) for item in candidates), default=0.0)
    limit = (1.0 / minimum_turning_radius) * (1.0 + relative_tolerance)
    violations = [item for item in candidates if abs(item["curvature_1pm"]) > limit]
    worst = max(violations, key=lambda item: abs(item["curvature_1pm"])) if violations else None
    variation = sum(
        abs(second["curvature_1pm"] - first["curvature_1pm"])
        for first, second in zip(candidates, candidates[1:])
        if first["group"] == second["group"]
    )
    return {
        "kinematically_feasible": not violations,
        "max_abs_curvature_1pm": maximum,
        "min_implied_radius_m": None if maximum <= 0.0 else 1.0 / maximum,
        "kinematic_violation_pose_index": None if worst is None else worst["pose_index"],
        "segment_length_m": None if worst is None else worst["segment_length_m"],
        "heading_delta_rad": None if worst is None else worst["heading_delta_rad"],
        "signed_direction": None if worst is None else worst["signed_direction"],
        "nearest_cusp_distance_m": None if worst is None else worst["nearest_cusp_distance_m"],
        "curvature_violation_count": len(violations),
        "cusps": len(cusp_vertices),
        "curvature_total_variation": variation,
        "curvature_samples": candidates,
    }


def transformed_footprint(x: float, y: float, yaw: float) -> tuple[tuple[float, float], ...]:
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((x + c * px - s * py, y + s * px + c * py) for px, py in FOOTPRINT)


def percentile(values: Iterable[float], quantile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    position = (len(ordered) - 1) * quantile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)
