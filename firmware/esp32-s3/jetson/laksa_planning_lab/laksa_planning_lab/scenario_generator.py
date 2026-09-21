"""Generate one persisted, deterministic scenario corpus shared by all methods."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random

from .geometry import auto_heading, wrap_pi
from .map_dataset import load_manifest, load_map

SEED = 2906
PRESETS = {"smoke": 8, "quick": 250, "baseline": 1000, "nightly": 5000}
DISTANCE_RANGES = {
    "VERY_NEAR": (0.5, 1.0), "NEAR": (1.0, 2.0),
    "MEDIUM": (2.0, 5.0), "FAR": (5.0, float("inf")),
}
BEARING_CENTERS = {
    "FRONT": 0.0, "LATERAL_LEFT": math.pi / 2,
    "LATERAL_RIGHT": -math.pi / 2, "BEHIND": math.pi,
}
SMOKE_STRATA = (
    ("FRONT", "NEAR", "OPEN_BASELINE"),
    ("FRONT", "MEDIUM", "STANDARD"),
    ("FRONT", "FAR", "STANDARD"),
    ("LATERAL_LEFT", "MEDIUM", "STANDARD"),
    ("LATERAL_RIGHT", "MEDIUM", "STANDARD"),
    ("BEHIND", "MEDIUM", "STANDARD"),
    ("FRONT", "MEDIUM", "OBSTACLE_DETOUR"),
    ("FRONT", "NEAR", "STANDARD"),
)


def scenario_targets(count: int) -> list[tuple[str, str, str]]:
    return [SMOKE_STRATA[index % len(SMOKE_STRATA)] for index in range(count)]


def distance_class(distance: float) -> str:
    for name, (low, high) in DISTANCE_RANGES.items():
        if low <= distance < high:
            return name
    return "TOO_CLOSE"


def bearing_class(relative_bearing: float) -> str:
    angle = wrap_pi(relative_bearing)
    if -math.pi / 4 <= angle < math.pi / 4:
        return "FRONT"
    if math.pi / 4 <= angle < 3 * math.pi / 4:
        return "LATERAL_LEFT"
    if -3 * math.pi / 4 <= angle < -math.pi / 4:
        return "LATERAL_RIGHT"
    return "BEHIND"


def _sample_safe_pose(map_data, rng: random.Random, attempts: int = 5000):
    for _ in range(attempts):
        mx, my = rng.randrange(map_data.width), rng.randrange(map_data.height)
        x, y = map_data.grid_to_world(mx, my)
        yaw = rng.uniform(-math.pi, math.pi)
        if map_data.safe_center(x, y) and map_data.pose_collision_free(x, y, yaw):
            return x, y, yaw
    raise RuntimeError(f"Unable to sample a SAFE GOAL pose on {map_data.map_id}")


def _try_stratum(map_data, rng, bearing_name, distance_name, kind, attempts=6000):
    low, high = DISTANCE_RANGES[distance_name]
    map_diagonal = math.hypot(map_data.width, map_data.height) * map_data.resolution
    high = min(high, map_diagonal * 0.85) if math.isfinite(high) else map_diagonal * 0.85
    if high <= low:
        return None
    for _ in range(attempts):
        try:
            sx, sy, syaw = _sample_safe_pose(map_data, rng, attempts=200)
        except RuntimeError:
            return None
        distance = rng.uniform(low, high)
        bearing = syaw + BEARING_CENTERS[bearing_name] + rng.uniform(-math.pi / 8, math.pi / 8)
        gx, gy = sx + distance * math.cos(bearing), sy + distance * math.sin(bearing)
        gyaw = auto_heading(sx, sy, syaw, gx, gy)
        start, goal = (sx, sy, syaw), (gx, gy, gyaw)
        if not (map_data.safe_center(gx, gy) and map_data.pose_collision_free(*goal)):
            continue
        straight_free = map_data.straight_path_collision_free(start, goal)
        if kind == "OPEN_BASELINE" and not straight_free:
            continue
        if kind == "OBSTACLE_DETOUR" and straight_free:
            continue
        rel = wrap_pi(bearing - syaw)
        if distance_class(distance) != distance_name or bearing_class(rel) != bearing_name:
            continue
        return start, goal, distance, rel, straight_free
    return None


def validate_scenario(scenario: dict, map_data) -> tuple[bool, str]:
    for label in ("start", "goal"):
        pose = scenario[label]
        if not map_data.safe_center(pose["x"], pose["y"]):
            return False, f"{label} does not satisfy conservative SAFE GOAL clearance"
        if not map_data.pose_collision_free(pose["x"], pose["y"], pose["yaw"]):
            return False, f"{label} footprint intersects unsafe map space"
    return True, ""


def generate_scenarios(manifest_path: Path, count: int, seed: int = SEED) -> dict:
    entries = load_manifest(manifest_path)
    if not entries:
        raise ValueError("Map manifest is empty")
    maps = [(entry, load_map(Path(entry["yaml"]), entry["map_id"])) for entry in entries]
    rng = random.Random(seed)
    targets = scenario_targets(count)
    scenarios = []
    for index, (requested_bearing, requested_distance, kind) in enumerate(targets):
        accepted = None
        offset = rng.randrange(len(maps))
        for map_offset in range(len(maps)):
            entry, map_data = maps[(offset + map_offset) % len(maps)]
            candidate = _try_stratum(map_data, rng, requested_bearing, requested_distance, kind)
            if candidate is not None:
                accepted = entry, map_data, candidate
                break
        if accepted is None:
            raise RuntimeError(
                f"Could not generate required {requested_bearing}/{requested_distance}/{kind} "
                "scenario from the available map corpus"
            )
        entry, map_data, (start, goal, distance, rel, straight_free) = accepted
        start_details = map_data.center_details(*start)
        goal_details = map_data.center_details(*goal)
        scenario = {
            "scenario_id": f"S{index:06d}", "map_id": entry["map_id"],
            "map_sha256": entry["sha256"],
            "start": {"x": start[0], "y": start[1], "yaw": start[2]},
            "goal": {"x": goal[0], "y": goal[1], "yaw": goal[2]},
            "goal_xy": {"x": goal[0], "y": goal[1]},
            "distance_class": distance_class(distance),
            "bearing_class": bearing_class(rel), "scenario_kind": kind,
            "straight_path_collision_free": straight_free,
            "required_center_clearance_m": map_data.required_center_clearance,
            "start_clearance_m": start_details["clearance_m"],
            "goal_clearance_m": goal_details["clearance_m"],
            "start_cell_cost": start_details["cell_cost"],
            "goal_cell_cost": goal_details["cell_cost"],
            "start_footprint_valid": start_details["footprint_valid"],
            "goal_footprint_valid": goal_details["footprint_valid"],
        }
        valid, reason = validate_scenario(scenario, map_data)
        if not valid:
            raise RuntimeError(f"Generated invalid scenario {scenario['scenario_id']}: {reason}")
        scenarios.append(scenario)
    return {
        "schema_version": 2, "seed": seed,
        "manifest": str(manifest_path.resolve()), "scenarios": scenarios,
    }


def save_scenarios(dataset: dict, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dataset, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_scenarios(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preset", choices=PRESETS, default="smoke")
    parser.add_argument("--count", type=int)
    parser.add_argument("--seed", type=int, default=SEED)
    args = parser.parse_args(argv)
    dataset = generate_scenarios(args.manifest, args.count or PRESETS[args.preset], args.seed)
    save_scenarios(dataset, args.output)
    print(f"wrote {len(dataset['scenarios'])} deterministic scenarios to {args.output}")


if __name__ == "__main__":
    main()
