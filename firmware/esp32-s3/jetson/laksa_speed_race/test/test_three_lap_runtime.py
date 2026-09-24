"""Atomic Gym-step and three-lap terminal behavior without ROS."""

import unittest
from pathlib import Path

from laksa_speed_race.gym_adapter_node import Command, GymStepAuthority
from laksa_speed_race.metrics import C1Metrics
from laksa_speed_race.three_lap_gate import MissionState


class AlwaysInside:
    def contains_body(self, x_m, y_m, yaw_rad):
        return True


class AlwaysOutside:
    def contains_body(self, x_m, y_m, yaw_rad):
        return False


class FakeGym:
    def __init__(self, laps=(1, 2, 3), collision=False):
        self.laps = list(laps)
        self.calls = 0
        self.collision = collision

    def step(self, action):
        lap = self.laps[self.calls]
        self.calls += 1
        obs = {
            "agent_0": {
                "std_state": [float(self.calls), 0.0, float(action[0][0]), float(action[0][1]), 0.0, 0.0, 0.0],
                "frenet_pose": [float(self.calls), 0.01, 0.02],
                "collision": self.collision,
            }
        }
        info = {"sim_time": self.calls * 10.0, "lap_counts": [lap], "lap_times": [10.0]}
        return obs, 0.01, lap == 3, False, info


class RuntimeGateTests(unittest.TestCase):
    def test_terminal_metadata_uses_persisted_package_share(self):
        source = Path(__file__).resolve().parents[1] / "laksa_speed_race" / "gym_adapter_node.py"
        text = source.read_text()
        self.assertIn("self.share = share", text)
        self.assertIn('_sha256(self.share / "config" / "c1_pure_pursuit.yaml")', text)
        self.assertNotIn('_sha256(share / "config" / "c1_pure_pursuit.yaml")', text)

    def test_exactly_three_steps_laps_and_terminal_zero(self):
        env = FakeGym()
        authority = GymStepAuthority(env, AlwaysInside(), C1Metrics(seed=12345))
        results = [authority.apply(Command(0.1, 1.0)) for _ in range(3)]
        self.assertEqual([result.lap_count for result in results], [1, 2, 3])
        self.assertTrue(results[-1].terminal)
        self.assertEqual(authority.gate.state, MissionState.STOPPING)
        self.assertEqual(authority.terminal_zero(), Command(0.0, 0.0))
        self.assertEqual(authority.gate.state, MissionState.COMPLETE)
        self.assertEqual(env.calls, 3)
        with self.assertRaises(RuntimeError):
            authority.apply(Command(0.0, 1.0))
        self.assertEqual(env.calls, 3)

    def test_reverse_fails_without_simulator_step(self):
        env = FakeGym()
        authority = GymStepAuthority(env, AlwaysInside(), C1Metrics(seed=12345))
        result = authority.apply(Command(0.0, -0.01))
        self.assertTrue(result.terminal)
        self.assertEqual(authority.gate.fault, "reverse_command")
        self.assertEqual(env.calls, 0)

    def test_out_of_limit_command_fails_without_step(self):
        env = FakeGym()
        authority = GymStepAuthority(env, AlwaysInside(), C1Metrics(seed=12345))
        result = authority.apply(Command(0.289, 1.0))
        self.assertTrue(result.terminal)
        self.assertEqual(authority.gate.fault, "command_outside_c1_limits")
        self.assertEqual(env.calls, 0)

    def test_collision_and_off_track_are_hard_faults(self):
        collision_env = FakeGym(laps=(0,), collision=True)
        collision = GymStepAuthority(collision_env, AlwaysInside(), C1Metrics(seed=12345))
        self.assertEqual(collision.apply(Command(0.0, 1.0)).fault, "collision")
        off_track_env = FakeGym(laps=(0,), collision=False)
        off_track = GymStepAuthority(off_track_env, AlwaysOutside(), C1Metrics(seed=12345))
        self.assertEqual(off_track.apply(Command(0.0, 1.0)).fault, "off_track")


if __name__ == "__main__":
    unittest.main()
