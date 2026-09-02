#!/usr/bin/env python3

"""Reproduce the 18-run noisy LAKSA kitchen acceptance campaign."""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parent / "laksa_bringup" / "scripts"
sys.path[:0] = [str(HERE), str(SCRIPTS)]

from ackermann_kitchen_sim import aggregate, simulate  # noqa: E402
from lidar_rollout_core import (  # noqa: E402
    AckermannRolloutController,
    RolloutParameters,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    results = []
    for scenario in ("island", "corner", "bottleneck"):
        for start in (0, 1):
            for seed in (11, 29, 47):
                results.append(
                    simulate(
                        AckermannRolloutController(RolloutParameters()),
                        scenario,
                        start,
                        duration=150.0,
                        seed=seed,
                        sensor_noise_std=0.012,
                        command_latency_sec=0.15,
                        steering_time_constant_sec=0.22,
                        odom_position_noise_std=0.008,
                        odom_yaw_noise_std=0.008,
                    )
                )
    report = {
        "conditions": {
            "runs": 18,
            "lidar_noise_std_m": 0.012,
            "command_latency_sec": 0.15,
            "steering_time_constant_sec": 0.22,
            "odom_position_noise_std_m": 0.008,
            "odom_yaw_noise_std_rad": 0.008,
        },
        "aggregate": aggregate(results),
        "runs": [result.json() for result in results],
    }
    encoded = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    metrics = report["aggregate"]
    if (
        metrics["targets_reached"] != 18
        or metrics["collisions"] != 0
        or metrics["minimum_coverage"] < 0.98
    ):
        raise SystemExit("Acceptance criteria failed")


if __name__ == "__main__":
    main()
