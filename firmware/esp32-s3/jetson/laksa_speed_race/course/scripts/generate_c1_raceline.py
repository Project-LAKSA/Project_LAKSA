#!/usr/bin/env python3
"""Generate C1 raceline assets with the pinned Waterloo/TUM helpers.

This LAKSA wrapper exists because the upstream script selects a hard-coded
minimum-time workflow and file layout.  It only translates the frozen course
contract into upstream calls and deterministic output formats; it contains no
optimizer, planner, controller, or simulator mathematics.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
CANONICAL = PACKAGE_ROOT / "course" / "canonical" / "speed_course"
OUTPUT_NAMES = (
    "speed_course_map.yaml",
    "speed_course_centerline.csv",
    "speed_course_raceline.csv",
    "pure_pursuit_raceline.csv",
)
# The pinned optimizer differs by up to 1e-9 between x86_64 and ARM64 for a
# handful of samples.  Seven decimal places retain sub-micrometre resolution
# while making the checked-in derived trajectory byte-reproducible across the
# two qualification architectures.  Authoritative source geometry is not
# quantized by this policy.
DERIVED_OUTPUT_DECIMAL_PLACES = 7


def format_derived_value(value: float) -> str:
    """Serialize optimizer-derived values at the cross-platform precision."""

    return f"{value:.{DERIVED_OUTPUT_DECIMAL_PLACES}f}"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_source_centerline(path: Path) -> np.ndarray:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    track = np.asarray(
        [
            (
                float(row["x_m"]),
                float(row["y_m"]),
                float(row["right_clearance_m"]),
                float(row["left_clearance_m"]),
            )
            for row in rows
        ],
        dtype=float,
    )
    if track.shape[0] < 4:
        raise ValueError("canonical centerline is too short")
    if np.linalg.norm(track[0, :2] - track[-1, :2]) <= 1e-9:
        track = track[:-1]
    return track


def upstream_modules(waterloo_root: Path):
    if not (waterloo_root / "helper_funcs_glob" / "src" / "prep_track.py").is_file():
        raise FileNotFoundError(f"invalid Waterloo raceline root: {waterloo_root}")
    sys.path.insert(0, str(waterloo_root))
    tph = importlib.import_module("trajectory_planning_helpers")
    prep_module = importlib.import_module("helper_funcs_glob.src.prep_track")
    # SciPy >=1.10 rejects the pinned helper's (2, 1) ``splev`` result where
    # older SciPy flattened it.  This is a shape-only compatibility adapter;
    # the distance and all upstream optimization mathematics are unchanged.
    spline_module = importlib.import_module("trajectory_planning_helpers.spline_approximation")

    def compatible_distance(t_glob, path, point):
        sample = np.asarray(spline_module.interpolate.splev(t_glob, path), dtype=float).reshape(-1)
        return spline_module.spatial.distance.euclidean(np.asarray(point).reshape(-1), sample)

    spline_module.dist_to_p = compatible_distance
    return prep_module.prep_track, tph


def write_outputs(output_dir: Path, source_track: np.ndarray, raceline: np.ndarray) -> dict[str, str]:
    output_dir.mkdir(parents=True, exist_ok=True)

    map_yaml = {
        "image": "speed_course_nav2.png",
        "resolution": 0.05,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    (output_dir / "speed_course_map.yaml").write_text(
        yaml.safe_dump(map_yaml, sort_keys=False), encoding="utf-8"
    )

    with (output_dir / "speed_course_centerline.csv").open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("x_m", "y_m", "w_tr_right_m", "w_tr_left_m"))
        for x_m, y_m, right_m, left_m in source_track:
            writer.writerow(tuple(f"{value:.9f}" for value in (x_m, y_m, right_m, left_m)))

    gym_path = output_dir / "speed_course_raceline.csv"
    with gym_path.open("w", newline="") as stream:
        stream.write("# LAKSA C1 deterministic Waterloo minimum-curvature raceline\n")
        stream.write("# source: canonical speed_course centerline; speed fixed at 1.0 m/s\n")
        stream.write("# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n")
        for row in raceline:
            stream.write("; ".join(format_derived_value(value) for value in row) + "\n")

    with (output_dir / "pure_pursuit_raceline.csv").open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        for row in raceline:
            writer.writerow(tuple(format_derived_value(value) for value in (row[1], row[2], row[5])))

    return {name: sha256(output_dir / name) for name in OUTPUT_NAMES}


def generate(course_dir: Path, output_dir: Path, waterloo_root: Path) -> dict[str, object]:
    config = yaml.safe_load((PACKAGE_ROOT / "config" / "c1_raceline.yaml").read_text())
    source_track = load_source_centerline(course_dir / "centerline.csv")
    prep_track, tph = upstream_modules(waterloo_root)

    steps = {
        "stepsize_prep": float(config["stepsize_prep_m"]),
        "stepsize_reg": float(config["stepsize_reg_m"]),
    }
    smoothing = {"k_reg": int(config["k_reg"]), "s_reg": float(config["s_reg"])}
    reftrack, normals, matrix_a, _, _ = prep_track(
        reftrack_imp=source_track,
        reg_smooth_opts=smoothing,
        stepsize_opts=steps,
        debug=False,
        min_width=None,
    )
    alpha, curvature_linearization_error = tph.opt_min_curv.opt_min_curv(
        reftrack=reftrack,
        normvectors=normals,
        A=matrix_a,
        kappa_bound=float(config["kappa_bound_1pm"]),
        w_veh=float(config["effective_vehicle_width_m"]),
        print_debug=False,
        plot_debug=False,
        closed=True,
    )
    (
        xy,
        _,
        coeffs_x,
        coeffs_y,
        spline_indices,
        spline_coordinates,
        arc_length,
        _,
        _,
    ) = tph.create_raceline.create_raceline(
        refline=reftrack[:, :2],
        normvectors=normals,
        alpha=alpha,
        stepsize_interp=float(config["stepsize_interp_after_opt_m"]),
    )
    psi_north, curvature = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
        ind_spls=spline_indices,
        t_spls=spline_coordinates,
    )
    # The pinned helper uses psi=0 toward north. Gym/ROS use psi=0 toward +X.
    psi_ros = (psi_north + math.pi / 2.0 + math.pi) % (2.0 * math.pi) - math.pi
    count = xy.shape[0]
    raceline = np.column_stack(
        (
            arc_length,
            xy[:, 0],
            xy[:, 1],
            psi_ros,
            curvature,
            np.full(count, float(config["target_speed_mps"])),
            np.full(count, float(config["target_acceleration_mps2"])),
        )
    )
    # Gym's periodic CubicSplineND requires one—and only one—terminal copy of
    # the first pose when an explicit s column is supplied.
    closing_length = float(np.linalg.norm(xy[-1] - xy[0]))
    terminal = raceline[0].copy()
    terminal[0] = raceline[-1, 0] + closing_length
    raceline = np.vstack((raceline, terminal))
    if not np.isfinite(raceline).all():
        raise ValueError("upstream raceline contains non-finite values")
    hashes = write_outputs(output_dir, source_track, raceline)
    return {
        "schema_version": "laksa-speed-race-c1-raceline-generation-v1",
        "status": "PASS",
        "sample_count": int(raceline.shape[0]),
        "closed_segment_m": 0.0,
        "maximum_abs_curvature_1pm": float(np.max(np.abs(curvature))),
        "curvature_linearization_error_1pm": float(curvature_linearization_error),
        "hashes": hashes,
    }


def update_manifest(course_dir: Path, result: dict[str, object]) -> None:
    manifest_path = course_dir / "course_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["raceline_contract"].update(
        {
            "output": "speed_course_raceline.csv: s_m,x_m,y_m,psi_rad,curvature_1pm,target_speed_mps,target_acceleration_mps2",
            "pure_pursuit_output": "pure_pursuit_raceline.csv: x_m,y_m,target_speed_mps",
            "status": "GENERATED_AND_VALIDATED",
            "upstream_raceline_sha": "9290c5d503462e46f7e3e9033002e7ddf165ba7b",
            "trajectory_helpers_sha": "fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56",
            "maximum_abs_curvature_1pm": result["maximum_abs_curvature_1pm"],
            "derived_output_decimal_places": DERIVED_OUTPUT_DECIMAL_PLACES,
        }
    )
    manifest["generated_asset_sha256"].update(result["hashes"])
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course-dir", type=Path, default=CANONICAL)
    parser.add_argument("--output-dir", type=Path, default=CANONICAL)
    parser.add_argument("--waterloo-root", type=Path, required=True)
    parser.add_argument("--update-manifest", action="store_true")
    args = parser.parse_args()
    result = generate(args.course_dir, args.output_dir, args.waterloo_root)
    if args.update_manifest:
        if args.output_dir.resolve() != args.course_dir.resolve():
            raise ValueError("manifest may only be updated for canonical output")
        update_manifest(args.course_dir, result)
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
