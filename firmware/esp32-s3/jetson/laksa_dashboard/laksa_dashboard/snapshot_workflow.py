"""Transactional, algorithm-free promotion of a completed mapping session.

This module never generates map content.  It validates and copies products
already produced by RTAB-Map, nav2_map_server and rtabmap-export.
"""

from __future__ import annotations

import datetime as dt
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import uuid

import yaml


class SnapshotError(RuntimeError):
    """Raised when a mapping session cannot be promoted atomically."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    tokens: list[bytes] = []
    with path.open("rb") as stream:
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
    if len(tokens) < 4 or tokens[0] not in (b"P2", b"P5"):
        raise SnapshotError(f"Invalid occupancy PGM: {path}")
    width, height = int(tokens[1]), int(tokens[2])
    if width <= 0 or height <= 0 or int(tokens[3]) <= 0:
        raise SnapshotError(f"Invalid occupancy PGM dimensions: {path}")
    return width, height


def _ply_vertex_count(path: Path) -> int:
    with path.open("rb") as stream:
        for raw in stream:
            line = raw.decode("ascii", errors="strict").strip()
            if line.startswith("element vertex "):
                count = int(line.rsplit(" ", 1)[1])
                if count <= 0:
                    raise SnapshotError("3D PLY has no vertices")
                return count
            if line == "end_header":
                break
    raise SnapshotError(f"PLY vertex declaration missing: {path}")


def _database_nodes(path: Path) -> int:
    try:
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            check = str(connection.execute("PRAGMA quick_check").fetchone()[0])
            nodes = int(connection.execute("SELECT COUNT(*) FROM Node").fetchone()[0])
    except (sqlite3.Error, TypeError, ValueError) as error:
        raise SnapshotError(f"RTAB database validation failed: {error}") from error
    if check != "ok" or nodes <= 0:
        raise SnapshotError(f"RTAB database invalid: check={check}, nodes={nodes}")
    return nodes


def create_snapshot(
    session_dir: Path,
    snapshots_root: Path,
    *,
    snapshot_id: str | None = None,
    final_pose: dict | None = None,
    runtime_context: dict | None = None,
) -> dict:
    """Atomically promote existing official mapping outputs to an immutable snapshot."""
    session_dir = session_dir.resolve()
    metadata_path = session_dir / "metadata.json"
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SnapshotError(f"Mapping metadata is unavailable: {error}") from error
    if metadata.get("result") != "COMPLETE" or not metadata.get("database_valid"):
        raise SnapshotError("Mapping session did not complete with a valid database")
    if not metadata.get("occupancy_exported") or not metadata.get("ply_exported"):
        raise SnapshotError("Mapping session exports are incomplete")

    sources = {
        "map.yaml": session_dir / "occupancy" / "map.yaml",
        "map.pgm": session_dir / "occupancy" / "map.pgm",
        "map.db": session_dir / "database" / "map.db",
        "metadata.json": metadata_path,
    }
    accepted_ply = session_dir / "export" / "map_cloud_cloud.ply"
    ply_candidates = sorted((session_dir / "export").glob("*.ply"))
    if accepted_ply.is_file():
        sources["map.ply"] = accepted_ply
    elif len(ply_candidates) == 1:
        sources["map.ply"] = ply_candidates[0]
    elif not ply_candidates:
        raise SnapshotError("Accepted 3D PLY is missing")
    else:
        raise SnapshotError("Accepted 3D PLY is ambiguous")
    missing = [name for name, path in sources.items() if not path.is_file() or path.stat().st_size == 0]
    if missing:
        raise SnapshotError("Required snapshot artifacts missing: " + ", ".join(missing))

    width, height = _pgm_dimensions(sources["map.pgm"])
    nodes = _database_nodes(sources["map.db"])
    points = _ply_vertex_count(sources["map.ply"])
    try:
        map_yaml = yaml.safe_load(sources["map.yaml"].read_text(encoding="utf-8")) or {}
        resolution = float(map_yaml["resolution"])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise SnapshotError(f"Invalid map.yaml: {error}") from error
    if resolution <= 0.0:
        raise SnapshotError("map.yaml has an invalid resolution")

    snapshots_root.mkdir(parents=True, exist_ok=True)
    snapshot_id = snapshot_id or dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    destination = snapshots_root / snapshot_id
    if destination.exists():
        raise SnapshotError(f"Snapshot already exists: {destination}")
    temporary = snapshots_root / f".{snapshot_id}.tmp-{uuid.uuid4().hex}"
    temporary.mkdir(mode=0o700)
    try:
        for name, source in sources.items():
            shutil.copy2(source, temporary / name)
        # Nav2 resolves relative image paths from map.yaml.  Normalize only the
        # copied descriptor; the accepted source session remains untouched.
        copied_yaml = yaml.safe_load((temporary / "map.yaml").read_text(encoding="utf-8")) or {}
        copied_yaml["image"] = "map.pgm"
        (temporary / "map.yaml").write_text(
            yaml.safe_dump(copied_yaml, sort_keys=False), encoding="utf-8"
        )
        artifact_hashes = {
            name: _sha256(temporary / name)
            for name in ("map.yaml", "map.pgm", "map.db", "map.ply", "metadata.json")
        }
        manifest = {
            "schema": 1,
            "snapshot_id": snapshot_id,
            "source_session": metadata.get("session_id", session_dir.name),
            "source_session_path": str(session_dir),
            "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "map_frame": "map",
            "map_resolution_m": resolution,
            "map_width_cells": width,
            "map_height_cells": height,
            "database_nodes": nodes,
            "ply_points": points,
            "configuration_hashes": metadata.get("configuration_hashes", {}),
            "final_pose": final_pose,
            "runtime": runtime_context or {},
            "artifact_sha256": artifact_hashes,
        }
        (temporary / "snapshot_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        sum_names = (*artifact_hashes.keys(), "snapshot_manifest.json")
        (temporary / "SHA256SUMS").write_text(
            "".join(f"{_sha256(temporary / name)}  {name}\n" for name in sum_names),
            encoding="utf-8",
        )
        temporary.rename(destination)
        for path in destination.iterdir():
            path.chmod(0o444)
        destination.chmod(0o555)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    current = snapshots_root / "current.json"
    current_tmp = snapshots_root / f".current-{uuid.uuid4().hex}.tmp"
    current_tmp.write_text(
        json.dumps({"snapshot_id": snapshot_id, "path": str(destination)}, indent=2) + "\n",
        encoding="utf-8",
    )
    current_tmp.replace(current)
    return {**manifest, "path": str(destination)}
