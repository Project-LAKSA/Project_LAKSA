"""Static C0 checks; these intentionally execute no ROS node or vehicle code."""

import json
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CompetitionC0Tests(unittest.TestCase):
    def test_manifest_uses_immutable_commit_refs(self):
        text = (ROOT / "speed_race_upstream.repos").read_text()
        versions = re.findall(r"^    version: ([0-9a-f]{40})$", text, re.MULTILINE)
        self.assertEqual(len(versions), 9)
        self.assertNotIn("main", versions)
        self.assertNotIn("master", versions)

    def test_competition_contract_excludes_manual_and_physical_control(self):
        text = (ROOT / "ARCHITECTURE.md").read_text().lower()
        self.assertIn("no joystick", text)
        self.assertIn("physical actuator", text)
        self.assertIn("/sim_ground_truth_map", text)

    def test_results_do_not_claim_unexecuted_simulation(self):
        results = json.loads((ROOT / "SPEED_RACE_RESULTS.json").read_text())
        self.assertEqual(results["C0"]["status"], "PASS")
        self.assertEqual(results["C1"]["status"], "PARTIAL")
        self.assertEqual(results["C1"]["simulation_run"], "NOT_EXECUTED")
        for phase in ("C2", "C3", "C4", "C5"):
            self.assertEqual(results[phase]["status"], "BLOCKED")
        self.assertEqual(results["repeat_runs"], 0)
        self.assertFalse(results["safety"]["physical_hardware_touched"])

    def test_sim_assumptions_keep_unknown_dynamics_unknown(self):
        assumptions = json.loads((ROOT / "SIM_ASSUMPTIONS.json").read_text())
        self.assertEqual(assumptions["course"]["status"], "CANONICAL_APPROVED_RECOVERED")
        self.assertTrue(assumptions["course"]["use_for_course_qualification"])
        self.assertFalse(assumptions["course"]["use_for_closed_loop_vehicle_qualification"])
        self.assertIn("tire_friction", assumptions["sim_assumed_not_physically_identified"])


if __name__ == "__main__":
    unittest.main()
