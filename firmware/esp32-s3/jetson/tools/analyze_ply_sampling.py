#!/usr/bin/env python3
"""Reproducible geometric audit of Field Lab sampling on a frozen RTAB-Map PLY."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree


def read_xyz(path: Path) -> np.ndarray:
    scalar = {
        "char": "i1", "uchar": "u1", "short": "<i2", "ushort": "<u2",
        "int": "<i4", "uint": "<u4", "float": "<f4", "double": "<f8",
        "int8": "i1", "uint8": "u1", "int16": "<i2", "uint16": "<u2",
        "int32": "<i4", "uint32": "<u4", "float32": "<f4", "float64": "<f8",
    }
    count = 0
    properties = []
    vertices = False
    with path.open("rb") as stream:
        if stream.readline().strip() != b"ply":
            raise ValueError("not a PLY")
        if stream.readline().strip() != b"format binary_little_endian 1.0":
            raise ValueError("expected binary_little_endian PLY")
        while True:
            line = stream.readline().decode("ascii").strip()
            fields = line.split()
            if fields[:2] == ["element", "vertex"]:
                count = int(fields[2]); vertices = True
            elif fields[:1] == ["element"]:
                vertices = False
            elif vertices and fields[:1] == ["property"] and len(fields) == 3:
                properties.append((fields[2], scalar[fields[1]]))
            if line == "end_header":
                offset = stream.tell(); break
    cloud = np.memmap(path, mode="r", dtype=np.dtype(properties), offset=offset, shape=(count,))
    xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float64)
    return xyz[np.all(np.isfinite(xyz), axis=1)]


def uniform_indices(count: int, target: int) -> np.ndarray:
    if count <= target:
        return np.arange(count, dtype=np.intp)
    return np.linspace(0, count - 1, target, dtype=np.intp)


def _part1by2(values: np.ndarray) -> np.ndarray:
    values = values.astype(np.uint64) & 0x3ff
    values = (values | values << 16) & 0x30000ff
    values = (values | values << 8) & 0x300f00f
    values = (values | values << 4) & 0x30c30c3
    values = (values | values << 2) & 0x9249249
    return values


def stratified_indices(xyz: np.ndarray, target: int) -> np.ndarray:
    if len(xyz) <= target:
        return np.arange(len(xyz), dtype=np.intp)
    minimum = xyz.min(axis=0); span = np.ptp(xyz, axis=0)
    span[span <= np.finfo(np.float64).eps] = 1.0
    quantized = np.minimum(((xyz - minimum) / span * 1024).astype(np.uint64), 1023)
    morton = (_part1by2(quantized[:, 0])
              | (_part1by2(quantized[:, 1]) << 1)
              | (_part1by2(quantized[:, 2]) << 2))
    ordered = np.argsort(morton, kind="stable")
    return np.sort(ordered[np.linspace(0, len(ordered) - 1, target, dtype=np.intp)])


def discover_planes(xyz: np.ndarray, seed: int = 20260913) -> list[dict]:
    rng = np.random.default_rng(seed)
    model_indices = uniform_indices(len(xyz), min(80000, len(xyz)))
    remaining = xyz[model_indices]
    planes = []
    for plane_number in range(3):
        if len(remaining) < 1000:
            break
        best = None
        for _ in range(180):
            tri = remaining[rng.choice(len(remaining), 3, replace=False)]
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            length = np.linalg.norm(normal)
            if length < 1.0e-8:
                continue
            normal /= length; offset = -normal @ tri[0]
            inliers = np.abs(remaining @ normal + offset) <= 0.04
            score = int(inliers.sum())
            if best is None or score > best[0]:
                best = (score, normal, offset, inliers)
        if best is None or best[0] < 500:
            break
        candidate = remaining[best[3]]
        center = candidate.mean(axis=0)
        _, _, vh = np.linalg.svd(candidate - center, full_matrices=False)
        normal = vh[-1]
        if normal[np.argmax(np.abs(normal))] < 0:
            normal = -normal
        offset = -normal @ center
        full_distance = xyz @ normal + offset
        inlier_count = int(np.count_nonzero(np.abs(full_distance) <= 0.05))
        planes.append({
            "id": plane_number + 1,
            "normal": normal.tolist(), "offset": float(offset),
            "full_inliers_5cm": inlier_count,
        })
        remaining = remaining[np.abs(remaining @ normal + offset) > 0.08]
    return planes


def plane_metrics(sample: np.ndarray, planes: list[dict]) -> list[dict]:
    result = []
    for plane in planes:
        normal = np.asarray(plane["normal"]); offset = float(plane["offset"])
        signed = sample @ normal + offset
        nearby = signed[np.abs(signed) <= 0.15]
        if not len(nearby):
            result.append({"id": plane["id"], "points_15cm": 0}); continue
        absolute = np.abs(nearby)
        hist, edges = np.histogram(nearby, bins=60, range=(-0.15, 0.15))
        peaks = np.argsort(hist)[-2:]
        separation = abs((edges[peaks[0]] + edges[peaks[0] + 1]) / 2
                         - (edges[peaks[1]] + edges[peaks[1] + 1]) / 2)
        result.append({
            "id": plane["id"], "points_15cm": int(len(nearby)),
            "inlier_ratio_5cm": round(float(np.mean(absolute <= 0.05)), 6),
            "rms_m": round(float(np.sqrt(np.mean(nearby * nearby))), 6),
            "p95_abs_m": round(float(np.percentile(absolute, 95)), 6),
            "robust_thickness_m": round(float(np.percentile(nearby, 95) - np.percentile(nearby, 5)), 6),
            "two_largest_histogram_peak_separation_m": round(float(separation), 6),
        })
    return result


def evaluate(full: np.ndarray, selected: np.ndarray, planes: list[dict], elapsed_ms: float) -> dict:
    sample = full[selected]
    rng = np.random.default_rng(20260913)
    probe = full[uniform_indices(len(full), min(50000, len(full)))]
    distances, _ = cKDTree(sample).query(probe, k=1, workers=1)
    cells = np.floor(sample / 0.05).astype(np.int64)
    occupied = len(np.unique(cells, axis=0))
    return {
        "sent_points": int(len(sample)), "sample_ms": round(elapsed_ms, 3),
        "estimated_compact_json_bytes": int(len(sample) * 6 * 7 + 256),
        "occupied_5cm_voxels": occupied,
        "nearest_sample_distance_p50_m": round(float(np.percentile(distances, 50)), 6),
        "nearest_sample_distance_p95_m": round(float(np.percentile(distances, 95)), 6),
        "planes": plane_metrics(sample, planes),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("ply", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--budgets", default="15000,30000,60000,0")
    parser.add_argument("--package-root", type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.package_root))
    from laksa_dashboard.spatial_sampling import spatial_voxel_sample_indices

    started = time.monotonic(); xyz = read_xyz(args.ply)
    planes = discover_planes(xyz)
    report = {
        "schema": "laksa-ply-sampling-audit-v1", "ply": str(args.ply),
        "source_points": int(len(xyz)), "load_and_plane_discovery_sec": round(time.monotonic() - started, 3),
        "plane_models": planes, "algorithms": {},
        "browser_fps": "NOT_RUN: requires live Field Lab browser instrumentation",
        "pose_latency": "NOT_RUN: requires live mapping session with browser",
    }
    budgets = [len(xyz) if int(value) == 0 else int(value) for value in args.budgets.split(",")]
    algorithms = {
        "current_uniform_input_order": lambda budget: uniform_indices(len(xyz), budget),
        "adaptive_voxel_representatives": lambda budget: spatial_voxel_sample_indices(xyz, budget),
        "morton_spatial_stratified": lambda budget: stratified_indices(xyz, budget),
    }
    for name, algorithm in algorithms.items():
        report["algorithms"][name] = {}
        for budget in budgets:
            before = time.monotonic(); selected = algorithm(budget); elapsed = (time.monotonic() - before) * 1000
            report["algorithms"][name][str(budget)] = evaluate(xyz, selected, planes, elapsed)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
