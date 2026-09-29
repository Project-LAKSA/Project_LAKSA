#!/usr/bin/env python3
"""Offline integrity and internal-consistency checks for C1.2e-R1 evidence."""

from __future__ import annotations

import csv
import hashlib
import json
from collections import Counter
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
EXPECTED_RAW_HASHES = {
    "replay_raw_telemetry.csv": "dffb8c4feeac98ae791f0b80e72ec3032b4e9704d412ed20c6d4cd054ab225ee",
    "critic_telemetry.csv": "766b7d97037cbc906566914c7bf59db327dbfe714e14eca185c88a8210842afe",
    "path_pipeline_telemetry.csv": "f799600e983d485c9e1d93234ac0e10810fbd1a2475831369cbc2f7285df8cb9",
    "replay_summary.json": "85add4645fbd49a69290db837ff093ec30bdee088a3c41462f3774882bf49fe3",
    "provenance.json": "0ce60cab01bf4721696cd0ede51f34a761b158202c16631515fd4f63ce50a4e8",
    "instrumentation_manifest.txt": "d38ea55c67c9b57e8f99aa1b03359c6d2d723a62737bb62e93bc96ebed5f2956",
    "instrumentation_nav2.diff": "360fc4fad4f19765f136959040fa380625e94325ad973b9209df2b5cbce44061",
}
FROZEN_FILES = {
    "controller_config_sha256":
        "firmware/esp32-s3/jetson/laksa_speed_race/config/c1_nav2_mppi.yaml",
    "proxy_config_sha256":
        "firmware/esp32-s3/jetson/laksa_speed_race/config/laksa_proxy_v0.yaml",
    "raceline_sha256":
        "firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/"
        "pure_pursuit_raceline.csv",
    "course_geometry_sha256":
        "firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/"
        "geometry.yaml",
    "course_source_sha256":
        "firmware/esp32-s3/jetson/laksa_speed_race/course/source/speed_course_full.jpeg",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def verify_manifest() -> int:
    path = HERE / "FORENSIC_MANIFEST.txt"
    checked = 0
    with path.open(newline="", encoding="utf-8") as stream:
        content = [line for line in stream if not line.startswith("#")]
        rows = csv.DictReader(content, delimiter="\t")
        for row in rows:
            artifact = HERE / row["relative_path"]
            assert artifact.is_file(), f"missing manifest file: {row['relative_path']}"
            assert artifact.stat().st_size == int(row["size_bytes"]), row["relative_path"]
            assert sha256(artifact) == row["sha256"], row["relative_path"]
            checked += 1
    return checked


def main() -> None:
    manifest_files = verify_manifest()
    for name, expected in EXPECTED_RAW_HASHES.items():
        assert sha256(HERE / name) == expected, name

    summary = json.loads((HERE / "replay_summary.json").read_text(encoding="utf-8"))
    provenance = json.loads((HERE / "provenance.json").read_text(encoding="utf-8"))
    assert summary["replay_reproduction"] == "PASS_EXACT"
    assert summary["metric_exact_equality"] is True
    assert summary["normalized_dynamic_hash_equal"] is True
    assert summary["total_steps"] == 1326
    assert summary["off_track_step"] == 1326
    assert summary["path_align_required_index"] == 20
    assert summary["path_align_max_available_index"] == 7
    assert summary["path_align_active_count"] == 0
    assert summary["path_align_inactive_count"] == 1326
    assert provenance["repository_sha"] == "7e280465090ee98e4470082cd8dff898e30d8377"
    assert provenance["nav2_mppi_sha"] == "a097086719c88f781aa59788eca29ac6ca5e56db"
    assert provenance["f1tenth_gym_sha"] == "bdaec1420c3b0f103858d289866d0d4e2e597c30"
    assert provenance["instrumentation"]["behavioral_effect"] == "NONE"

    replay = csv_rows(HERE / "replay_raw_telemetry.csv")
    critics = csv_rows(HERE / "critic_telemetry.csv")
    paths = csv_rows(HERE / "path_pipeline_telemetry.csv")
    assert len(replay) == len(paths) == 1326
    assert len(critics) == 1326 * 8
    assert [int(row["step_index"]) for row in replay] == list(range(1, 1327))
    assert [int(row["evaluation"]) for row in paths] == list(range(1, 1327))
    assert int(replay[-1]["off_track"]) == 1
    assert sum(int(row["off_track"]) for row in replay) == 1
    assert all(row["path_align_activation_state"] == "INACTIVE" for row in paths)
    assert max(int(row["path_align_max_available_index"]) for row in paths) == 7
    critic_counts = Counter(row["critic_name"] for row in critics)
    assert len(critic_counts) == 8
    assert set(critic_counts.values()) == {1326}

    frozen = provenance["frozen_configuration_hashes"]
    for key, relative in FROZEN_FILES.items():
        assert sha256(REPO / relative) == frozen[key], relative

    result = {
        "artifact_provenance": "PASS",
        "manifest_files_verified": manifest_files,
        "raw_artifact_hashes": "PASS",
        "replay_reproduction": summary["replay_reproduction"],
        "total_steps": len(replay),
        "off_track_step": int(replay[-1]["step_index"]),
        "critic_rows": len(critics),
        "path_align_active_count": 0,
        "path_align_inactive_count": len(paths),
        "frozen_configuration_hashes": "PASS",
        "new_simulation_run": "NO",
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
