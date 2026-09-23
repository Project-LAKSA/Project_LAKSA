"""LAKSA_PROXY_V0 and fixed C1 configuration contract."""

import math
import unittest
from pathlib import Path

import yaml

from laksa_speed_race.c1_contract import DT_S, MAX_LAPS, MAX_SPEED_MPS, SEED, STEERING_LIMIT_RAD


ROOT = Path(__file__).resolve().parents[1]


class ProxyContractTests(unittest.TestCase):
    def test_proxy_matches_reviewed_values(self):
        config = yaml.safe_load((ROOT / "config" / "laksa_proxy_v0.yaml").read_text())
        proxy = config["laksa_proxy_v0"]
        simulation = config["simulation"]
        self.assertEqual(proxy["model"], "KS")
        self.assertEqual(proxy["wheelbase_m"], 0.324)
        self.assertEqual(proxy["body_length_m"], 0.568)
        self.assertEqual(proxy["body_width_m"], 0.296)
        self.assertEqual(proxy["collision_body_center_m"], [0.135, 0.0])
        self.assertEqual(proxy["steering_limit_rad"], STEERING_LIMIT_RAD)
        self.assertEqual(proxy["max_speed_mps"], MAX_SPEED_MPS)
        self.assertEqual(simulation["dt_s"], DT_S)
        self.assertEqual(simulation["seed"], SEED)
        self.assertEqual(simulation["max_laps"], MAX_LAPS)
        self.assertEqual(simulation["integrator"], "RK4")

    def test_curvature_derivation_is_conservative(self):
        expected = math.tan(STEERING_LIMIT_RAD) / 0.324
        self.assertAlmostEqual(expected, 0.91431, places=5)

    def test_dynamic_defaults_are_labeled_assumptions(self):
        config = yaml.safe_load((ROOT / "config" / "laksa_proxy_v0.yaml").read_text())
        self.assertEqual(config["simulation_assumptions"]["classification"], "SIMULATION_ASSUMPTION")


if __name__ == "__main__":
    unittest.main()
