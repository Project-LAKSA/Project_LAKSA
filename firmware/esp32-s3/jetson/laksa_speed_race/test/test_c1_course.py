"""Offline C1 course, mission, metrics, and simulator-only boundary tests."""

import csv
import unittest
from pathlib import Path

import yaml

from laksa_speed_race.c1_contract import (
    CONTROLLER_REQUEST_TOPIC,
    FORBIDDEN_PHYSICAL_COMMAND_TOPICS,
    SIMULATOR_APPLIED_TOPIC,
    SIMULATOR_ODOM_TOPIC,
)
from laksa_speed_race.course_validation import (
    required_full_body_clearance_m,
    validate,
)
from laksa_speed_race.metrics import C1Metrics
from laksa_speed_race.three_lap_gate import MissionState, ThreeLapGate


ROOT = Path(__file__).resolve().parents[1]
COURSE = ROOT / "course" / "canonical" / "speed_course"


class C1CourseTests(unittest.TestCase):
    def test_gym_uses_canonical_high_resolution_raster(self):
        map_config = yaml.safe_load((COURSE / "speed_course_map.yaml").read_text())
        self.assertEqual(map_config["image"], "speed_course_hires.png")
        self.assertEqual(map_config["resolution"], 0.02)

    def test_recovered_course_contract(self):
        result = validate(ROOT / "course")
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["checks"]["raceline"], "PASS")
        self.assertEqual(result["checks"]["raceline_body_envelope"], "PASS")
        self.assertEqual(result["checks"]["raceline_full_body_clearance"], "PASS")
        self.assertGreaterEqual(
            result["raceline_clearance"]["minimum_full_body_clearance_m"],
            result["raceline_clearance"]["required_clearance_m"],
        )

    def test_previous_4mm_clearance_regression_fails_closed(self):
        import json

        manifest = json.loads((COURSE / "course_manifest.json").read_text())
        required = required_full_body_clearance_m(manifest)
        self.assertLess(0.004017015281351566, required)
        self.assertGreater(required, 0.005)

    def test_external_raceline_input_is_lossless_and_headerless(self):
        from course.scripts.export_raceline_input import export

        exported = export()
        with exported.open(newline="") as stream:
            rows = list(csv.reader(stream))
        self.assertEqual(len(rows), 4446)
        self.assertEqual(len(rows[0]), 4)
        self.assertEqual(rows[0], ["20.628630908", "2.298687755", "0.457200000", "0.457200000"])

    def test_three_laps_then_only_zero_propulsion(self):
        gate = ThreeLapGate()
        gate.ready()
        gate.start()
        for duration in (10.0, 9.5, 9.0):
            gate.record_lap(duration)
        self.assertEqual(gate.state, MissionState.STOPPING)
        self.assertFalse(gate.propulsion_permitted)
        gate.record_lap(8.0)
        self.assertEqual(gate.state, MissionState.FAULT)
        self.assertEqual(gate.fault, "lap_event_outside_running")

        gate = ThreeLapGate()
        gate.ready()
        gate.start()
        for duration in (10.0, 9.5, 9.0):
            gate.record_lap(duration)
        gate.stop_confirmed()
        self.assertEqual(gate.state, MissionState.COMPLETE)
        self.assertTrue(gate.done)
        self.assertEqual(gate.lap_count, 3)

    def test_early_done_faults_closed(self):
        gate = ThreeLapGate()
        gate.ready()
        gate.start()
        gate.stop_confirmed()
        self.assertEqual(gate.state, MissionState.FAULT)
        self.assertEqual(gate.fault, "simulator_done_before_three_laps")
        self.assertFalse(gate.propulsion_permitted)

    def test_metrics_never_invents_cte_threshold(self):
        metrics = C1Metrics(seed=7)
        metrics.cross_track_errors_m.extend((0.1, -0.2, 0.3))
        metrics.record_command(1.0, 0.288)
        metrics.record_command(-0.1, 0.0)
        summary = metrics.summary(lap_times_s=[1.0, 1.0, 1.0], sim_time_s=3.0, done=True)
        self.assertEqual(summary["cte_pass_threshold"], "UNSET")
        self.assertEqual(summary["reverse_command_events"], 1)
        self.assertEqual(summary["steering_saturation_events"], 1)

    def test_c1_contract_cannot_route_to_physical_topics(self):
        self.assertEqual(CONTROLLER_REQUEST_TOPIC, "/c1/drive_request")
        self.assertEqual(SIMULATOR_APPLIED_TOPIC, "/c1/drive_applied")
        self.assertEqual(SIMULATOR_ODOM_TOPIC, "/c1/odom")
        self.assertEqual(
            FORBIDDEN_PHYSICAL_COMMAND_TOPICS,
            {"/drive", "/laksa/command", "/cmd_vel", "/laksa/set_drive_command"},
        )
        runtime_sources = list((ROOT / "laksa_speed_race").glob("*.py"))
        for source in runtime_sources:
            text = source.read_text()
            self.assertNotIn("micro_ros_bridge", text)
            self.assertNotIn("VESC", text)
            self.assertNotIn("GPIO", text)


if __name__ == "__main__":
    unittest.main()
