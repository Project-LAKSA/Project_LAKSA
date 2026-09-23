"""C1 simulation metric aggregation without pass/fail CTE threshold invention."""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower, upper = math.floor(index), math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


@dataclass
class C1Metrics:
    """Record values supplied by an upstream simulator/controller integration."""

    seed: int
    cross_track_errors_m: list[float] = field(default_factory=list)
    heading_errors_rad: list[float] = field(default_factory=list)
    speed_commands_mps: list[float] = field(default_factory=list)
    steering_commands_rad: list[float] = field(default_factory=list)
    trajectory: list[tuple[float, float, float]] = field(default_factory=list)
    steering_saturation_events: int = 0
    collision_edges: int = 0
    off_track_events: int = 0
    reverse_command_events: int = 0

    def record_command(self, speed_mps: float, steering_rad: float) -> None:
        self.speed_commands_mps.append(speed_mps)
        self.steering_commands_rad.append(steering_rad)
        if speed_mps < 0.0:
            self.reverse_command_events += 1
        if abs(steering_rad) >= 0.288:
            self.steering_saturation_events += 1

    def summary(self, *, lap_times_s: list[float], sim_time_s: float, done: bool) -> dict[str, object]:
        absolute_cte = [abs(value) for value in self.cross_track_errors_m]
        cte_rms = math.sqrt(sum(value * value for value in self.cross_track_errors_m) / len(self.cross_track_errors_m)) if self.cross_track_errors_m else None
        return {
            "schema_version": "laksa-speed-race-c1-metrics-v1",
            "seed": self.seed,
            "lap_times_s": lap_times_s,
            "lap_count": len(lap_times_s),
            "total_time_s": sum(lap_times_s),
            "sim_time_s": sim_time_s,
            "done": done,
            "collision_edges": self.collision_edges,
            "off_track_events": self.off_track_events,
            "reverse_command_events": self.reverse_command_events,
            "steering_saturation_events": self.steering_saturation_events,
            "cte_mean_m": sum(self.cross_track_errors_m) / len(self.cross_track_errors_m) if self.cross_track_errors_m else None,
            "cte_rms_m": cte_rms,
            "cte_p95_m": _percentile(absolute_cte, 0.95),
            "cte_max_m": max(absolute_cte) if absolute_cte else None,
            "cte_pass_threshold": "UNSET",
        }
