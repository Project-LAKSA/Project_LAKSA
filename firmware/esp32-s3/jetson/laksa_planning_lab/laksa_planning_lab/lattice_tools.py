"""Generate and independently validate conservative/asymmetric Nav2 lattices."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

from .geometry import R_LEFT_M, R_RIGHT_M, signed_curvatures, wrap_pi

REVERSE_FINDING = "EXACT_ASYMMETRIC_REVERSE_LATTICE_UNSUPPORTED_BY_STOCK_HUMBLE_NODELATTICE"


def find_humble_generator() -> Path:
    candidates = [
        Path("/opt/ros/humble/share/nav2_smac_planner/lattice_primitives/generate_motion_primitives.py"),
        Path("/opt/ros/humble/lib/python3.10/site-packages/nav2_smac_planner/lattice_primitives/generate_motion_primitives.py"),
        Path("/home/ubuntu/third_party/third_party_ws/src/navigation2/nav2_smac_planner/lattice_primitives/generate_motion_primitives.py"),
        Path("/home/ubuntu/src/navigation2/nav2_smac_planner/lattice_primitives/generate_motion_primitives.py"),
    ]
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix("nav2_smac_planner"))
        candidates.extend(prefix.glob("**/lattice_primitives/generate_motion_primitives.py"))
    except ImportError:
        pass
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError("ROS 2 Humble Nav2 lattice generator not found")


def _official_generate(generator: Path, radius: float, resolution: float, headings: int, threshold: int, output: Path) -> dict:
    output.parent.mkdir(parents=True, exist_ok=True)
    config = {"motion_model": "ackermann", "turning_radius": radius, "grid_resolution": resolution,
              "stopping_threshold": threshold, "num_of_headings": headings}
    config_path = output.with_suffix(".config.json")
    visualizations = output.parent / (output.stem + "_visualizations")
    config_path.write_text(json.dumps(config, sort_keys=True), encoding="utf-8")
    subprocess.run([sys.executable, str(generator), "--config", str(config_path), "--output", str(output),
                    "--visualizations", str(visualizations)], cwd=generator.parent, check=True)
    data = json.loads(output.read_text(encoding="utf-8"))
    data["date_generated"] = "1970-01-01"
    return data


def _primitive_key(primitive: dict):
    return (primitive["start_angle_index"], primitive["end_angle_index"],
            tuple(tuple(round(float(v), 10) for v in pose) for pose in primitive["poses"]))


def validate_lattice(data: dict, left_radius: float, right_radius: float, tolerance: float = 0.03) -> dict:
    metadata = data.get("lattice_metadata", {})
    headings = metadata.get("heading_angles", [])
    primitives = data.get("primitives", [])
    errors = []
    if not headings or metadata.get("number_of_trajectories") != len(primitives):
        errors.append("invalid metadata trajectory count/headings")
    for key in ("turning_radius", "grid_resolution"):
        if not math.isfinite(float(metadata.get(key, float("nan")))):
            errors.append(f"non-finite metadata {key}")
    ids = [primitive.get("trajectory_id") for primitive in primitives]
    if ids != list(range(len(primitives))):
        errors.append("trajectory IDs are not unique sequential IDs")
    previous_sort_key = None
    for primitive in primitives:
        start_index, end_index = primitive.get("start_angle_index"), primitive.get("end_angle_index")
        if not isinstance(start_index, int) or not isinstance(end_index, int) or not (0 <= start_index < len(headings) and 0 <= end_index < len(headings)):
            errors.append(f"primitive {primitive.get('trajectory_id')}: invalid heading index"); continue
        poses = primitive.get("poses")
        if not poses or any(len(pose) != 3 or any(not math.isfinite(float(value)) for value in pose) for pose in poses):
            errors.append(f"primitive {primitive.get('trajectory_id')}: empty/non-finite poses"); continue
        numeric_fields = ("trajectory_radius", "trajectory_length", "arc_length", "straight_length")
        if any(not math.isfinite(float(primitive.get(field, float("nan")))) for field in numeric_fields):
            errors.append(f"primitive {primitive['trajectory_id']}: non-finite numeric field"); continue
        if math.hypot(float(poses[0][0]), float(poses[0][1])) > 1e-5:
            errors.append(f"primitive {primitive['trajectory_id']}: start is not origin")
        if abs(wrap_pi(float(poses[0][2]) - float(headings[start_index]))) > 1e-4 or abs(wrap_pi(float(poses[-1][2]) - float(headings[end_index]))) > 1e-3:
            errors.append(f"primitive {primitive['trajectory_id']}: endpoint heading mismatch")
        if float(primitive.get("trajectory_length", 0.0)) <= 0.0:
            errors.append(f"primitive {primitive['trajectory_id']}: non-positive length")
        resolution = float(metadata.get("grid_resolution", 0.0))
        if resolution > 0 and (abs(float(poses[-1][0]) / resolution - round(float(poses[-1][0]) / resolution)) > 1e-3 or
                               abs(float(poses[-1][1]) / resolution - round(float(poses[-1][1]) / resolution)) > 1e-3):
            errors.append(f"primitive {primitive['trajectory_id']}: endpoint is off lattice grid")
        if any(abs(wrap_pi(float(b[2]) - float(a[2]))) > math.pi / 2 for a, b in zip(poses, poses[1:])):
            errors.append(f"primitive {primitive['trajectory_id']}: yaw discontinuity")
        radius = float(primitive.get("trajectory_radius", 0.0))
        if abs(float(primitive.get("arc_length", 0.0))) > 1e-8:
            required = left_radius if bool(primitive.get("left_turn")) else right_radius
            if radius + tolerance < required:
                errors.append(f"primitive {primitive['trajectory_id']}: radius {radius} below {required}")
            curvatures = signed_curvatures(poses)
            limit = 1.0 / required
            if curvatures and max(abs(value) for value in curvatures) > limit * 1.10 + 1e-6:
                errors.append(f"primitive {primitive['trajectory_id']}: sampled curvature exceeds limit")
        sort_key = (start_index, end_index, 0 if bool(primitive.get("left_turn")) else 1, _primitive_key(primitive)[2])
        if previous_sort_key is not None and sort_key < previous_sort_key:
            errors.append("primitives are not deterministically sorted")
        previous_sort_key = sort_key
    return {"valid": not errors, "primitive_count": len(primitives), "errors": errors}


def generate_lattices(output_dir: Path, resolution: float, headings: int = 16, threshold: int = 5, generator: Path | None = None) -> dict:
    generator = generator or find_humble_generator()
    key = f"{resolution:.6f}".replace(".", "p")
    output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="laksa_lattice_") as temp_name:
        temp = Path(temp_name)
        left = _official_generate(generator, R_LEFT_M, resolution, headings, threshold, temp / "left.json")
        right = _official_generate(generator, R_RIGHT_M, resolution, headings, threshold, temp / "right.json")
    if left["lattice_metadata"]["heading_angles"] != right["lattice_metadata"]["heading_angles"]:
        raise ValueError("Official generators produced incompatible heading metadata")
    conservative = copy.deepcopy(right)
    conservative["date_generated"] = "1970-01-01"
    conservative["primitives"].sort(key=lambda primitive: (primitive["start_angle_index"], primitive["end_angle_index"],
                                                            0 if primitive["left_turn"] else 1, _primitive_key(primitive)[2]))
    for index, primitive in enumerate(conservative["primitives"]): primitive["trajectory_id"] = index
    conservative_path = output_dir / f"conservative_{key}.json"
    # Straight controls are radius-independent. Take exactly one conservative
    # copy; then take each curved steering side from its measured radius set.
    primitives = [copy.deepcopy(value) for value in right["primitives"] if abs(float(value.get("arc_length", 0.0))) <= 1e-8]
    primitives.extend(copy.deepcopy(value) for value in left["primitives"]
                      if abs(float(value.get("arc_length", 0.0))) > 1e-8 and bool(value["left_turn"]))
    primitives.extend(copy.deepcopy(value) for value in right["primitives"]
                      if abs(float(value.get("arc_length", 0.0))) > 1e-8 and not bool(value["left_turn"]))
    primitives.sort(key=lambda primitive: (primitive["start_angle_index"], primitive["end_angle_index"],
                                            0 if primitive["left_turn"] else 1, _primitive_key(primitive)[2]))
    for index, primitive in enumerate(primitives): primitive["trajectory_id"] = index
    asymmetric = copy.deepcopy(right)
    asymmetric["date_generated"] = "1970-01-01"
    asymmetric["lattice_metadata"]["turning_radius"] = max(R_LEFT_M, R_RIGHT_M)
    asymmetric["lattice_metadata"]["number_of_trajectories"] = len(primitives)
    asymmetric["primitives"] = primitives
    asymmetric_path = output_dir / f"asymmetric_forward_{key}.json"
    validations = {
        "conservative": validate_lattice(conservative, R_RIGHT_M, R_RIGHT_M),
        "asymmetric_forward": validate_lattice(asymmetric, R_LEFT_M, R_RIGHT_M),
    }
    if not all(value["valid"] for value in validations.values()):
        raise ValueError("Lattice validation failed: " + json.dumps(validations, sort_keys=True))
    conservative_path.write_text(json.dumps(conservative, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    asymmetric_path.write_text(json.dumps(asymmetric, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report = {"conservative": str(conservative_path), "asymmetric_forward": str(asymmetric_path), "validation": validations,
              "r_left_m": R_LEFT_M, "r_right_m": R_RIGHT_M, "grid_resolution": resolution,
              "sha256": {"conservative": hashlib.sha256(conservative_path.read_bytes()).hexdigest(),
                         "asymmetric_forward": hashlib.sha256(asymmetric_path.read_bytes()).hexdigest()},
              "reverse_finding": REVERSE_FINDING}
    (output_dir / f"validation_{key}.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--resolution", type=float, required=True)
    parser.add_argument("--headings", type=int, default=16)
    args = parser.parse_args(argv)
    print(json.dumps(generate_lattices(args.output_dir, args.resolution, args.headings), indent=2))
