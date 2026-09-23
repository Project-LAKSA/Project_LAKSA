"""C1 evidence persistence and strict acceptance tests."""

import json
import tempfile
import unittest
from pathlib import Path

from laksa_speed_race.metrics import C1Metrics
from laksa_speed_race.run_validation import persist_run
from laksa_speed_race.three_lap_gate import ThreeLapGate


class MetricsTests(unittest.TestCase):
    def test_complete_evidence_and_no_cte_threshold(self):
        metrics = C1Metrics(seed=12345)
        gate = ThreeLapGate()
        gate.ready()
        gate.start()
        for index, lap_time in enumerate((10.0, 9.9, 9.8), start=1):
            metrics.record_step(
                step=index,
                sim_time_s=sum((10.0, 9.9, 9.8)[:index]),
                requested_speed_mps=1.0,
                requested_steering_rad=0.1,
                applied_speed_mps=1.0,
                applied_steering_rad=0.1,
                x_m=float(index),
                y_m=0.0,
                yaw_rad=0.0,
                cte_m=0.01,
                heading_error_rad=0.02,
                collision=False,
                off_track=False,
                lap_count=index,
                state="RUNNING",
            )
            gate.record_lap(lap_time)
        gate.record_terminal_zero(0.0, 0.0)
        with tempfile.TemporaryDirectory() as directory:
            summary = persist_run(
                Path(directory),
                metrics=metrics,
                gate=gate,
                sim_time_s=29.7,
                metadata={"test": True},
                final_requested_command={"steering_rad": 0.1, "speed_mps": 1.0},
                final_applied_command={"steering_rad": 0.0, "speed_mps": 0.0},
            )
            self.assertEqual(summary["acceptance"]["overall"], "PASS")
            self.assertEqual(summary["cte_pass_threshold"], "UNSET")
            self.assertTrue((Path(directory) / "trajectory.csv").is_file())
            self.assertTrue((Path(directory) / "commands.csv").is_file())
            loaded = json.loads((Path(directory) / "summary.json").read_text())
            self.assertEqual(loaded["completed_laps"], 3)


if __name__ == "__main__":
    unittest.main()
