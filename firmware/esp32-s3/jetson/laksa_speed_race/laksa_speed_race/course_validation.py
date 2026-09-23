"""Offline validator for the recovered, frozen LAKSA Speed Course.

The validator checks provenance, hashes, vector/raster consistency and the
vehicle-envelope contract. It intentionally contains no map generation,
raceline optimizer, controller, simulator, ROS node, or physical command path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import struct
import sys
import zlib
from pathlib import Path
from typing import Any


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_COURSE_ROOT = PACKAGE_ROOT / "course"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _finite(value: float, label: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"non-finite {label}")


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)
    return (ab_c > 0) != (ab_d > 0) and (cd_a > 0) != (cd_b > 0)


def _assert_no_self_intersection(points: list[tuple[float, float]], cell_size_m: float = 0.5) -> None:
    """Check a closed polyline with a spatial grid; adjacent segments may touch."""
    segment_count = len(points) - 1
    cells: dict[tuple[int, int], list[int]] = {}
    for index in range(segment_count):
        a, b = points[index], points[index + 1]
        min_x, max_x = sorted((a[0], b[0]))
        min_y, max_y = sorted((a[1], b[1]))
        for x in range(math.floor(min_x / cell_size_m), math.floor(max_x / cell_size_m) + 1):
            for y in range(math.floor(min_y / cell_size_m), math.floor(max_y / cell_size_m) + 1):
                for other in cells.setdefault((x, y), []):
                    if abs(index - other) <= 1 or {index, other} == {0, segment_count - 1}:
                        continue
                    if _segments_intersect(a, b, points[other], points[other + 1]):
                        raise ValueError(f"self intersection between centerline segments {other} and {index}")
                cells[(x, y)].append(index)


def _png_gray(path: Path) -> tuple[int, int, list[bytes]]:
    """Read the project’s non-interlaced 8-bit grayscale PNG without Pillow."""
    raw = path.read_bytes()
    if raw[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"invalid PNG signature: {path.name}")
    position = 8
    width = height = bit_depth = color_type = interlace = None
    compressed = bytearray()
    while position < len(raw):
        length = struct.unpack(">I", raw[position : position + 4])[0]
        chunk_type = raw[position + 4 : position + 8]
        data = raw[position + 8 : position + 8 + length]
        position += 12 + length
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, compression, filtering, interlace = struct.unpack(">IIBBBBB", data)
            if (bit_depth, color_type, compression, filtering, interlace) != (8, 0, 0, 0, 0):
                raise ValueError(f"unsupported PNG encoding: {path.name}")
        elif chunk_type == b"IDAT":
            compressed.extend(data)
        elif chunk_type == b"IEND":
            break
    if width is None or height is None:
        raise ValueError(f"missing PNG header: {path.name}")
    encoded = zlib.decompress(compressed)
    rows: list[bytes] = []
    previous = bytearray(width)
    offset = 0
    for _ in range(height):
        filter_type = encoded[offset]
        offset += 1
        scanline = bytearray(encoded[offset : offset + width])
        offset += width
        for index in range(width):
            left = scanline[index - 1] if index else 0
            up = previous[index]
            up_left = previous[index - 1] if index else 0
            if filter_type == 1:
                scanline[index] = (scanline[index] + left) & 0xFF
            elif filter_type == 2:
                scanline[index] = (scanline[index] + up) & 0xFF
            elif filter_type == 3:
                scanline[index] = (scanline[index] + ((left + up) // 2)) & 0xFF
            elif filter_type == 4:
                predictor = left + up - up_left
                choices = (left, up, up_left)
                scanline[index] = (scanline[index] + min(choices, key=lambda value: abs(predictor - value))) & 0xFF
            elif filter_type != 0:
                raise ValueError(f"unsupported PNG filter: {filter_type}")
        rows.append(bytes(scanline))
        previous = scanline
    return width, height, rows


def _pixel_at(rows: list[bytes], resolution: float, x: float, y: float) -> int:
    height, width = len(rows), len(rows[0])
    column = min(max(int(x / resolution), 0), width - 1)
    row = min(max(height - 1 - int(y / resolution), 0), height - 1)
    return rows[row][column]


def _load_centerline(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    if len(rows) < 4:
        raise ValueError("centerline has too few samples")
    for row in rows:
        for key, value in row.items():
            _finite(value, f"centerline.{key}")
    return rows


def validate(course_root: Path | None = None) -> dict[str, Any]:
    course_root = course_root or DEFAULT_COURSE_ROOT
    canonical = course_root / "canonical" / "speed_course"
    manifest = json.loads((canonical / "course_manifest.json").read_text())
    checks: dict[str, str] = {}

    if manifest["schema_version"] != "laksa-speed-race-course-manifest-v1":
        raise ValueError("unexpected course manifest schema")
    if manifest["approval_status"] != "CANONICAL_APPROVED" or not manifest["geometry_frozen"]:
        raise ValueError("course is not a frozen approved asset")
    if manifest["units"] != {"length": "m", "angle": "rad", "time": "s"}:
        raise ValueError("course units are not explicit SI units")
    if manifest["coordinate_frame"]["name"] != "speed_course_map":
        raise ValueError("course coordinate frame is not explicit")
    if manifest["overall_dimensions_m"] != {"length": 41.148, "width": 14.325600000000001}:
        raise ValueError("course dimensions differ from the approved blueprint conversion")
    if manifest["track_width_m"] != 0.9144:
        raise ValueError("course width differs from the approved 36-inch path")
    if manifest["start_finish"]["crossing_direction"] != "+X":
        raise ValueError("start/finish direction is absent or incorrect")
    checks["manifest"] = "PASS"

    for name, expected in manifest["source_files"].items():
        if _sha256(course_root / "source" / name) != expected:
            raise ValueError(f"source hash mismatch: {name}")
    for name, expected in manifest["generated_asset_sha256"].items():
        if _sha256(canonical / name) != expected:
            raise ValueError(f"generated asset hash mismatch: {name}")
    checks["hashes"] = "PASS"

    report = json.loads((canonical / "validation_report.json").read_text())
    if report["validation_result"] != "CANONICAL_APPROVED" or report["closed_route_error_m"] != 0.0:
        raise ValueError("historical reconstruction report is not approved and closed")
    if not report["topology_checks"]["all_route_samples_free"]:
        raise ValueError("historical raster/vector route check failed")
    checks["historical_raster_validation"] = "PASS"

    centerline = _load_centerline(canonical / "centerline.csv")
    points = [(row["x_m"], row["y_m"]) for row in centerline]
    if _distance(points[0], points[-1]) > 1e-9:
        raise ValueError("centerline is not closed")
    if any(b["s_m"] <= a["s_m"] for a, b in zip(centerline, centerline[1:])):
        raise ValueError("centerline arc length is not strictly increasing")
    if max(_distance(a, b) for a, b in zip(points, points[1:])) > 0.05:
        raise ValueError("centerline sampling exceeds contract")
    if max(abs(row["curvature_1pm"]) for row in centerline) > manifest["laksa_proxy_v0"]["max_curvature_1pm"]:
        raise ValueError("centerline exceeds conservative LAKSA curvature capability")
    _assert_no_self_intersection(points)
    checks["centerline"] = "PASS"

    boundaries = json.loads((canonical / "boundaries.geojson").read_text())
    features = boundaries.get("features", [])
    if boundaries.get("coordinate_units") != "meters" or len(features) != 2:
        raise ValueError("boundary representation is invalid")
    boundary_points: list[list[tuple[float, float]]] = []
    for feature in features:
        if feature["geometry"]["type"] != "LineString":
            raise ValueError("boundary is not a line string")
        line = [(float(x), float(y)) for x, y in feature["geometry"]["coordinates"]]
        if len(line) != len(points) or _distance(line[0], line[-1]) > 1e-9:
            raise ValueError("boundary is not a valid closed line")
        boundary_points.append(line)
    for point, left, right in zip(points, boundary_points[0], boundary_points[1]):
        midpoint = ((left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0)
        width = _distance(left, right)
        if _distance(point, midpoint) > 1e-6 or abs(width - manifest["track_width_m"]) > 1e-6:
            raise ValueError("centerline does not remain inside the authoritative corridor")
    checks["boundaries"] = "PASS"

    nav2 = canonical / "speed_course_nav2.png"
    png_width, png_height, rows = _png_gray(nav2)
    expected_width = math.ceil(manifest["overall_dimensions_m"]["length"] / 0.05)
    expected_height = math.ceil(manifest["overall_dimensions_m"]["width"] / 0.05)
    if (png_width, png_height) != (expected_width, expected_height):
        raise ValueError("occupancy dimensions do not match course dimensions")
    half_width = manifest["track_width_m"] / 2.0
    for row in centerline[::20]:
        x, y, yaw = row["x_m"], row["y_m"], row["yaw_rad"]
        normal = (-math.sin(yaw), math.cos(yaw))
        if _pixel_at(rows, 0.05, x, y) <= 127:
            raise ValueError("occupancy raster blocks the authoritative centerline")
        for side in (-1.0, 1.0):
            outside = (x + side * normal[0] * (half_width + 0.10), y + side * normal[1] * (half_width + 0.10))
            if _pixel_at(rows, 0.05, *outside) > 127:
                raise ValueError("occupancy raster does not preserve an authoritative barrier")
    checks["occupancy_vector_agreement"] = "PASS"

    raceline = canonical / "raceline.csv"
    checks["raceline"] = "PASS" if raceline.exists() else "UPSTREAM_GENERATION_PENDING"
    return {"status": "PASS", "course": manifest["course_id"], "checks": checks}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course-root", type=Path, default=DEFAULT_COURSE_ROOT)
    args = parser.parse_args()
    print(json.dumps(validate(args.course_root), sort_keys=True))


if __name__ == "__main__":
    try:
        main()
    except ValueError as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}), file=sys.stderr)
        raise SystemExit(1)
