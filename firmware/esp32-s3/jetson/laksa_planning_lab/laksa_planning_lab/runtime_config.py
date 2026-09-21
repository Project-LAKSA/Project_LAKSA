"""Render one immutable Nav2 configuration for one isolated trial process."""

from __future__ import annotations

import copy
from pathlib import Path
import yaml


METHOD_FILES = {
    "HYBRID_PRODUCTION": "hybrid_production.yaml",
    "HYBRID_RAW": "hybrid_raw.yaml",
    "HYBRID_CONSTRAINED": "hybrid_constrained.yaml",
    "LATTICE_CONSERVATIVE": "lattice_conservative.yaml",
    "LATTICE_ASYMMETRIC_FORWARD": "lattice_asymmetric_forward.yaml",
}


def _merge(target: dict, changes: dict):
    for key, value in changes.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict): _merge(target[key], value)
        else: target[key] = value


def render(package_share: Path, method: str, map_resolution: float, output: Path, overrides: dict | None = None, lattice_file: str = "") -> dict:
    source = package_share / "config" / METHOD_FILES[method]
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    data["/laksa_planning_lab/global_costmap/global_costmap"]["ros__parameters"]["resolution"] = float(map_resolution)
    plugin = data["/laksa_planning_lab/planner_server"]["ros__parameters"]["GridBased"]
    if lattice_file:
        plugin["lattice_filepath"] = str(Path(lattice_file).resolve())
    if overrides:
        _merge(plugin, copy.deepcopy(overrides.get("planner", {})))
        if method == "HYBRID_CONSTRAINED":
            _merge(data["/laksa_planning_lab/smoother_server"]["ros__parameters"]["SmoothPath"], copy.deepcopy(overrides.get("smoother", {})))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return data
