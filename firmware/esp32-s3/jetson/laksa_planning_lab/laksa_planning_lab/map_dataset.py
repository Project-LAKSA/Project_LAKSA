"""Read-only discovery and loading of map_server-compatible LAKSA maps."""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import struct
from typing import Iterator

import yaml

from .geometry import FOOTPRINT, transformed_footprint


def representative_cell_indices(cells: list[int]) -> list[int]:
    """Choose deterministic spatial and semantic samples for map_server parity."""
    if not cells:
        return []
    indices = {0, len(cells) // 4, len(cells) // 2, 3 * len(cells) // 4, len(cells) - 1}
    for value in (-1, 0, 100):
        try:
            indices.add(cells.index(value))
            indices.add(len(cells) - 1 - cells[::-1].index(value))
        except ValueError:
            pass
    return sorted(indices)


def _edt_1d(values: list[float]) -> list[float]:
    """Exact squared Euclidean distance transform."""
    size = len(values)
    sites = [index for index, value in enumerate(values) if math.isfinite(value)]
    if not sites:
        return [math.inf] * size
    vertices = [0] * len(sites)
    boundaries = [0.0] * (len(sites) + 1)
    k = 0
    vertices[0] = sites[0]
    boundaries[0], boundaries[1] = -math.inf, math.inf
    for site in sites[1:]:
        while True:
            prior = vertices[k]
            crossing = (
                (values[site] + site * site) - (values[prior] + prior * prior)
            ) / (2.0 * (site - prior))
            if crossing > boundaries[k] or k == 0:
                break
            k -= 1
        k += 1
        vertices[k] = site
        boundaries[k], boundaries[k + 1] = crossing, math.inf
    result = [0.0] * size
    k = 0
    for index in range(size):
        while boundaries[k + 1] < index:
            k += 1
        delta = index - vertices[k]
        result[index] = delta * delta + values[vertices[k]]
    return result


class MapData:
    def __init__(self, map_id: str, yaml_path: Path, metadata: dict, pixels: list[int], width: int, height: int):
        self.map_id, self.yaml_path, self.metadata = map_id, yaml_path, metadata
        self.width, self.height = width, height
        self.resolution = float(metadata["resolution"])
        self.origin = tuple(float(v) for v in metadata.get("origin", (0, 0, 0)))
        mode = str(metadata.get("mode", "trinary")).lower()
        if mode != "trinary":
            raise ValueError(f"Unsupported map mode {mode!r}; Planning Lab V1.1 requires map_server trinary semantics")
        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.196))
        self.cells: list[int] = []
        for pixel in pixels:
            probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
            self.cells.append(100 if probability > occupied_thresh else (0 if probability < free_thresh else -1))
        self.clearance = self._distance_field()
        self.circumscribed_radius = max(math.hypot(x, y) for x, y in FOOTPRINT)
        self.required_center_clearance = self.circumscribed_radius + self.resolution * math.sqrt(0.5)

    def index(self, mx: int, my: int) -> int:
        return my * self.width + mx

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        # map_server map origins may include yaw; transform world into map-image axes.
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        dx, dy = x - ox, y - oy
        return int(math.floor((c * dx + s * dy) / self.resolution)), int(math.floor((-s * dx + c * dy) / self.resolution))

    def grid_to_world(self, mx: int, my: int) -> tuple[float, float]:
        return self._grid_coordinate_to_world(mx + 0.5, my + 0.5)

    def _grid_coordinate_to_world(self, gx: float, gy: float) -> tuple[float, float]:
        ox, oy, oyaw = self.origin
        gx, gy = gx * self.resolution, gy * self.resolution
        c, s = math.cos(oyaw), math.sin(oyaw)
        return ox + c * gx - s * gy, oy + s * gx + c * gy

    def valid_point(self, x: float, y: float) -> bool:
        mx, my = self.world_to_grid(x, y)
        return 0 <= mx < self.width and 0 <= my < self.height and self.cells[self.index(mx, my)] == 0

    def clearance_at(self, x: float, y: float) -> float:
        mx, my = self.world_to_grid(x, y)
        if not (0 <= mx < self.width and 0 <= my < self.height):
            return 0.0
        return self.clearance[self.index(mx, my)]

    def cell_cost_at(self, x: float, y: float) -> int | None:
        mx, my = self.world_to_grid(x, y)
        if not (0 <= mx < self.width and 0 <= my < self.height):
            return None
        return self.cells[self.index(mx, my)]

    def safe_center(self, x: float, y: float) -> bool:
        return (
            self.cell_cost_at(x, y) == 0
            and self.clearance_at(x, y) + 1.0e-12 >= self.required_center_clearance
        )

    def center_details(self, x: float, y: float, yaw: float) -> dict:
        return {
            "clearance_m": self.clearance_at(x, y),
            "cell_cost": self.cell_cost_at(x, y),
            "footprint_valid": self.pose_collision_free(x, y, yaw),
            "safe_center": self.safe_center(x, y),
        }

    @staticmethod
    def _point_in_polygon(px: float, py: float, polygon) -> bool:
        inside = False
        previous = polygon[-1]
        for current in polygon:
            if ((current[1] > py) != (previous[1] > py)) and px < ((previous[0] - current[0]) * (py - current[1]) / (previous[1] - current[1]) + current[0]):
                inside = not inside
            previous = current
        return inside

    @staticmethod
    def _segments_intersect(a, b, c, d) -> bool:
        def cross(p, q, r): return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        def on_segment(p, q, r):
            return min(p[0], r[0]) - 1e-12 <= q[0] <= max(p[0], r[0]) + 1e-12 and min(p[1], r[1]) - 1e-12 <= q[1] <= max(p[1], r[1]) + 1e-12
        abc, abd, cda, cdb = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
        if abc * abd < 0 and cda * cdb < 0: return True
        return ((abs(abc) <= 1e-12 and on_segment(a, c, b)) or (abs(abd) <= 1e-12 and on_segment(a, d, b)) or
                (abs(cda) <= 1e-12 and on_segment(c, a, d)) or (abs(cdb) <= 1e-12 and on_segment(c, b, d)))

    @classmethod
    def _cell_intersects_polygon(cls, corners, polygon) -> bool:
        if any(cls._point_in_polygon(*corner, polygon) for corner in corners): return True
        min_x, max_x = min(point[0] for point in corners), max(point[0] for point in corners)
        min_y, max_y = min(point[1] for point in corners), max(point[1] for point in corners)
        if any(min_x <= point[0] <= max_x and min_y <= point[1] <= max_y for point in polygon): return True
        return any(cls._segments_intersect(a, b, c, d) for a, b in zip(polygon, polygon[1:] + polygon[:1])
                   for c, d in zip(corners, corners[1:] + corners[:1]))

    def pose_collision_free(self, x: float, y: float, yaw: float) -> bool:
        return self.pose_collision_evidence(x, y, yaw)["collision_free"]

    def pose_collision_evidence(self, x: float, y: float, yaw: float) -> dict:
        """Check the actual oriented LAKSA rectangle and retain offending cells."""
        polygon = transformed_footprint(x, y, yaw)
        grid = [self.world_to_grid(px, py) for px, py in polygon]
        min_x, max_x = min(v[0] for v in grid) - 1, max(v[0] for v in grid) + 1
        min_y, max_y = min(v[1] for v in grid) - 1, max(v[1] for v in grid) + 1
        offending = []
        for my in range(min_y, max_y + 1):
            for mx in range(min_x, max_x + 1):
                corners = [self._grid_coordinate_to_world(mx, my), self._grid_coordinate_to_world(mx + 1, my),
                           self._grid_coordinate_to_world(mx + 1, my + 1), self._grid_coordinate_to_world(mx, my + 1)]
                if not self._cell_intersects_polygon(corners, polygon):
                    continue
                in_bounds = 0 <= mx < self.width and 0 <= my < self.height
                cost = self.cells[self.index(mx, my)] if in_bounds else None
                if not in_bounds or cost != 0:
                    offending.append({"mx": mx, "my": my, "occupancy": cost})
        return {
            "collision_free": not offending,
            "footprint_polygon": [[px, py] for px, py in polygon],
            "offending_cells": offending,
            "reason": "" if not offending else (
                "footprint_outside_map" if any(cell["occupancy"] is None for cell in offending)
                else "footprint_intersects_unknown_or_occupied_cell"
            ),
        }

    def path_collision_report(self, poses, spacing: float | None = None) -> dict:
        spacing = spacing or min(self.resolution * 0.5, 0.025)
        if not poses:
            return {"collision_free": False, "reason": "empty_path", "pose_index": None}
        samples = []
        for pose_index, (first, second) in enumerate(zip(poses, poses[1:])):
            distance = math.hypot(second[0] - first[0], second[1] - first[1])
            count = max(1, int(math.ceil(distance / spacing)))
            for step in range(count):
                fraction = step / count
                yaw_delta = (second[2] - first[2] + math.pi) % (2 * math.pi) - math.pi
                samples.append((pose_index, fraction, (
                    first[0] + fraction * (second[0] - first[0]),
                    first[1] + fraction * (second[1] - first[1]),
                    first[2] + fraction * yaw_delta,
                )))
        samples.append((len(poses) - 1, 0.0, poses[-1]))
        for pose_index, fraction, pose in samples:
            evidence = self.pose_collision_evidence(*pose)
            if not evidence["collision_free"]:
                evidence.update({
                    "pose_index": pose_index, "sample_fraction": fraction,
                    "x": pose[0], "y": pose[1], "yaw": pose[2],
                })
                return evidence
        return {"collision_free": True, "reason": "", "pose_index": None}

    def path_collision_free(self, poses, spacing: float | None = None) -> bool:
        return self.path_collision_report(poses, spacing)["collision_free"]

    def straight_path_collision_free(self, start, goal) -> bool:
        return self.path_collision_free((start, goal))

    def _distance_field(self) -> list[float]:
        # Match Field Lab SAFE GOAL: unknown/occupied/outside are unsafe and
        # Euclidean clearance is exact at cell centers.
        padded_width, padded_height = self.width + 2, self.height + 2
        blocked = [[0.0] * padded_width for _ in range(padded_height)]
        for row in range(1, padded_height - 1):
            for column in range(1, padded_width - 1):
                cell = self.cells[self.index(column - 1, row - 1)]
                blocked[row][column] = math.inf if cell == 0 else 0.0
        horizontal = [_edt_1d(row) for row in blocked]
        squared = [[0.0] * padded_width for _ in range(padded_height)]
        for column in range(padded_width):
            transformed = _edt_1d([horizontal[row][column] for row in range(padded_height)])
            for row, value in enumerate(transformed):
                squared[row][column] = value
        return [
            math.sqrt(squared[row + 1][column + 1]) * self.resolution
            for row in range(self.height) for column in range(self.width)
        ]


def _read_pgm(path: Path) -> tuple[list[int], int, int]:
    raw = path.read_bytes()
    index, tokens = 0, []
    while len(tokens) < 4:
        while index < len(raw) and chr(raw[index]).isspace(): index += 1
        if index < len(raw) and raw[index] == 35:
            index = raw.index(b"\n", index) + 1
            continue
        start = index
        while index < len(raw) and not chr(raw[index]).isspace(): index += 1
        tokens.append(raw[start:index].decode("ascii"))
    magic, width, height, maximum = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == "P5":
        if index >= len(raw) or not chr(raw[index]).isspace():
            raise ValueError(f"Malformed P5 header in {path}")
        index += 2 if raw[index:index + 2] == b"\r\n" else 1
        byte_count = width * height * (1 if maximum < 256 else 2)
        raster = raw[index:index + byte_count]
        if len(raster) != byte_count:
            raise ValueError(f"Truncated P5 raster in {path}")
        values = list(raster) if maximum < 256 else list(struct.unpack(f">{width * height}H", raster))
    elif magic == "P2":
        while index < len(raw) and chr(raw[index]).isspace(): index += 1
        values = [int(value) for value in raw[index:].split()]
    else:
        raise ValueError(f"Unsupported map image {magic}; export a PGM map")
    if maximum != 255:
        values = [round(value * 255 / maximum) for value in values]
    # PGM rows are top-to-bottom; OccupancyGrid rows are bottom-to-top.
    rows = [values[row * width:(row + 1) * width] for row in range(height)]
    return [value for row in reversed(rows) for value in row], width, height


def load_map(yaml_path: Path, map_id: str | None = None) -> MapData:
    yaml_path = yaml_path.resolve()
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image = Path(str(metadata["image"]))
    image = image if image.is_absolute() else yaml_path.parent / image
    if image.suffix.lower() != ".pgm":
        raise ValueError(f"{image}: V1 supports PGM maps; re-export this session with map_saver_cli")
    pixels, width, height = _read_pgm(image)
    return MapData(map_id or yaml_path.parent.parent.name, yaml_path, metadata, pixels, width, height)


def discover_maps(sessions_root: Path) -> list[dict]:
    entries = []
    for yaml_path in sorted(sessions_root.glob("*/occupancy/map.yaml")):
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image = Path(str(metadata["image"]))
        image = image if image.is_absolute() else yaml_path.parent / image
        if not image.is_file():
            continue
        digest = hashlib.sha256(yaml_path.read_bytes() + b"\0" + image.read_bytes()).hexdigest()
        try:
            loaded = load_map(yaml_path)
        except ValueError:
            continue
        entries.append({"map_id": yaml_path.parent.parent.name, "source_session_id": yaml_path.parent.parent.name,
                        "yaml": str(yaml_path.resolve()), "resolution": loaded.resolution,
                        "width": loaded.width, "height": loaded.height, "sha256": digest})
    return entries


def write_manifest(entries: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({"maps": entries}, sort_keys=False), encoding="utf-8")


def load_manifest(path: Path) -> list[dict]:
    return list((yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("maps", []))


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Discover read-only LAKSA occupancy maps")
    parser.add_argument("--sessions-root", type=Path, default=Path("/home/ubuntu/laksa_mapping_sessions"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    entries = discover_maps(args.sessions_root)
    if not entries:
        raise SystemExit("No map_server-compatible real LAKSA PGM maps found; export a completed mapping session first")
    write_manifest(entries, args.output)
    print(f"wrote {len(entries)} real-map entries to {args.output}")
