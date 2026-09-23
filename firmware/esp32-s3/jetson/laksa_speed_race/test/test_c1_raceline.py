"""Generated C1 raceline provenance and geometry contracts."""

import csv
import hashlib
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COURSE = ROOT / "course" / "canonical" / "speed_course"


class RacelineTests(unittest.TestCase):
    def test_generated_hashes_match_manifest(self):
        manifest = json.loads((COURSE / "course_manifest.json").read_text())
        for name in (
            "speed_course_map.yaml",
            "speed_course_centerline.csv",
            "speed_course_raceline.csv",
            "pure_pursuit_raceline.csv",
        ):
            digest = hashlib.sha256((COURSE / name).read_bytes()).hexdigest()
            self.assertEqual(digest, manifest["generated_asset_sha256"][name])
        self.assertEqual(manifest["raceline_contract"]["status"], "GENERATED_AND_VALIDATED")

    def test_pure_pursuit_artifact_is_forward_only(self):
        with (COURSE / "pure_pursuit_raceline.csv").open(newline="") as stream:
            rows = [[float(value) for value in row] for row in csv.reader(stream)]
        self.assertGreater(len(rows), 100)
        self.assertTrue(all(len(row) == 3 for row in rows))
        self.assertTrue(all(row[2] == 1.0 for row in rows))
        self.assertEqual(rows[0][:2], rows[-1][:2])
        self.assertNotEqual(rows[0][:2], rows[-2][:2])

    def test_exact_upstream_pins(self):
        text = (ROOT / "speed_race_upstream.repos").read_text()
        self.assertIn("9290c5d503462e46f7e3e9033002e7ddf165ba7b", text)
        self.assertIn("fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56", text)


if __name__ == "__main__":
    unittest.main()
