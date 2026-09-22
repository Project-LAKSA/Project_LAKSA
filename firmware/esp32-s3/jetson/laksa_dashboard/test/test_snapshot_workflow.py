import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

import yaml

from laksa_dashboard.snapshot_workflow import SnapshotError, create_snapshot


def _session(root: Path, *, complete: bool = True) -> Path:
    session = root / "session"
    for child in ("occupancy", "database", "export"):
        (session / child).mkdir(parents=True, exist_ok=True)
    (session / "occupancy" / "map.pgm").write_bytes(b"P5\n3 2\n255\n" + bytes(range(6)))
    (session / "occupancy" / "map.yaml").write_text(
        yaml.safe_dump({"image": "map.pgm", "resolution": 0.05, "origin": [0, 0, 0]}),
        encoding="utf-8",
    )
    with sqlite3.connect(session / "database" / "map.db") as connection:
        connection.execute("CREATE TABLE Node(id INTEGER PRIMARY KEY)")
        connection.execute("INSERT INTO Node VALUES(1)")
    (session / "export" / "cloud.ply").write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 1 1\n",
        encoding="ascii",
    )
    (session / "metadata.json").write_text(
        json.dumps({
            "result": "COMPLETE" if complete else "ERROR",
            "database_valid": complete,
            "occupancy_exported": complete,
            "ply_exported": complete,
            "session_id": "session",
            "configuration_hashes": {"rtabmap.yaml": "abc"},
        }),
        encoding="utf-8",
    )
    return session


class SnapshotWorkflowTest(unittest.TestCase):
    def test_snapshot_is_transactional_versioned_and_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = create_snapshot(
                _session(root), root / "snapshots", snapshot_id="20260913T000000Z",
                final_pose={"x": 1.0, "y": 2.0, "yaw": 0.3},
                runtime_context={"rmw": "rmw_cyclonedds_cpp"},
            )
            snapshot = Path(result["path"])
            self.assertEqual(snapshot.name, "20260913T000000Z")
            self.assertEqual(result["database_nodes"], 1)
            self.assertEqual(result["ply_points"], 2)
            self.assertEqual(result["map_width_cells"], 3)
            self.assertEqual(result["map_height_cells"], 2)
            self.assertTrue(
                {"map.yaml", "map.pgm", "map.db", "map.ply", "metadata.json", "SHA256SUMS", "snapshot_manifest.json"}
                <= {path.name for path in snapshot.iterdir()}
            )
            self.assertEqual(yaml.safe_load((snapshot / "map.yaml").read_text())["image"], "map.pgm")
            self.assertEqual(json.loads((root / "snapshots" / "current.json").read_text())["path"], str(snapshot))
            self.assertEqual(snapshot.stat().st_mode & 0o222, 0)
            self.assertFalse(list((root / "snapshots").glob(".*.tmp-*")))

    def test_snapshot_rejects_incomplete_session_without_partial_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(SnapshotError, "did not complete"):
                create_snapshot(
                    _session(root, complete=False), root / "snapshots",
                    snapshot_id="20260913T000001Z",
                )
            self.assertFalse((root / "snapshots" / "20260913T000001Z").exists())


if __name__ == "__main__":
    unittest.main()
