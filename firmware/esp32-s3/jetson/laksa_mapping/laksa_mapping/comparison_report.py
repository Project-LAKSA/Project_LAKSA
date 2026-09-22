#!/usr/bin/env python3
"""Render a factual ZED-only versus hybrid-shadow session comparison."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .shadow_policy import HYBRID_SHADOW, MAPPING_SOURCES, ZED_ONLY


FIELDS = (
    ("Session", "session_id"),
    ("Result", "result"),
    ("Duration (s)", "elapsed_sec"),
    ("RTAB nodes", "database_nodes"),
    ("Map width (cells)", "map_width_cells"),
    ("Map height (cells)", "map_height_cells"),
    ("Occupied cells", "occupied_cells"),
    ("Free cells", "free_cells"),
    ("Unknown cells", "unknown_cells"),
    ("Occupancy updates (Hz)", "occupancy_update_rate_hz"),
    ("RTAB mean time", "rtab_processing_ms_mean"),
    ("RTAB max time", "rtab_processing_ms_max"),
    ("Validated scan (Hz)", "validated_scan_rate_hz"),
    ("LiDAR health", "lidar_health"),
)


def _flatten(data: dict) -> dict:
    result = dict(data.get("comparison_metrics", {}))
    result.update(data)
    return result


def latest_sessions(root: Path) -> dict[str, dict]:
    latest = {}
    for path in sorted(root.glob("*/metadata.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        source = data.get("mapping_source")
        if source in MAPPING_SOURCES and data.get("result") in ("COMPLETE", "ERROR"):
            latest[source] = _flatten(data)
    return latest


def render(sessions: dict[str, dict]) -> str:
    columns = (ZED_ONLY, HYBRID_SHADOW)
    lines = [
        "# LAKSA A030A Mapping A/B Comparison", "",
        "No synthetic map-quality score is calculated.", "",
        f"| Metric | {columns[0]} | {columns[1]} |", "|---|---:|---:|",
    ]
    for label, key in FIELDS:
        values = [sessions.get(source, {}).get(key, "not recorded") for source in columns]
        lines.append(f"| {label} | {values[0]} | {values[1]} |")
    lines.extend(["", "CPU usage is not synthesized; record it externally during controlled A/B runs."])
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sessions-root", type=Path, default=Path("/home/ubuntu/laksa_mapping_sessions"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = render(latest_sessions(args.sessions_root))
    if args.output:
        args.output.write_text(report, encoding="utf-8")
    else:
        print(report, end="")


if __name__ == "__main__":
    main()
