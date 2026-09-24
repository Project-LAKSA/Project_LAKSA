"""C1 simulation metric aggregation without pass/fail CTE threshold invention."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


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
    invalid_command_events: int = 0
    simulator_steps: int = 0
    command_rows: list[dict[str, float | int | str]] = field(default_factory=list)
    trajectory_rows: list[dict[str, float | int]] = field(default_factory=list)
    _collision_active: bool = False
    _off_track_active: bool = False

    def record_command(self, speed_mps: float, steering_rad: float) -> None:
        self.speed_commands_mps.append(speed_mps)
        self.steering_commands_rad.append(steering_rad)
        if speed_mps < 0.0:
            self.reverse_command_events += 1
        if abs(steering_rad) >= 0.288:
            self.steering_saturation_events += 1

    def record_step(
        self,
        *,
        step: int,
        sim_time_s: float,
        requested_speed_mps: float,
        requested_steering_rad: float,
        applied_speed_mps: float,
        applied_steering_rad: float,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        cte_m: float,
        heading_error_rad: float,
        collision: bool,
        off_track: bool,
        lap_count: int,
        state: str,
    ) -> None:
        self.simulator_steps += 1
        self.record_command(requested_speed_mps, requested_steering_rad)
        self.cross_track_errors_m.append(cte_m)
        self.heading_errors_rad.append(heading_error_rad)
        self.trajectory.append((x_m, y_m, yaw_rad))
        if collision and not self._collision_active:
            self.collision_edges += 1
        self._collision_active = collision
        if off_track and not self._off_track_active:
            self.off_track_events += 1
        self._off_track_active = off_track
        self.command_rows.append(
            {
                "step": step,
                "sim_time_s": sim_time_s,
                "requested_steering_rad": requested_steering_rad,
                "requested_speed_mps": requested_speed_mps,
                "applied_steering_rad": applied_steering_rad,
                "applied_speed_mps": applied_speed_mps,
                "mission_state": state,
            }
        )
        self.trajectory_rows.append(
            {
                "step": step,
                "sim_time_s": sim_time_s,
                "x_m": x_m,
                "y_m": y_m,
                "yaw_rad": yaw_rad,
                "signed_cte_m": cte_m,
                "heading_error_rad": heading_error_rad,
                "collision": int(collision),
                "off_track": int(off_track),
                "lap_count": lap_count,
            }
        )

    def summary(
        self,
        *,
        lap_times_s: list[float],
        sim_time_s: float,
        done: bool,
        metadata: dict[str, Any] | None = None,
        terminal_state: str | None = None,
        final_requested_command: dict[str, float] | None = None,
        final_applied_command: dict[str, float] | None = None,
        steps_after_terminal: int = 0,
    ) -> dict[str, object]:
        absolute_cte = [abs(value) for value in self.cross_track_errors_m]
        absolute_heading = [abs(value) for value in self.heading_errors_rad]
        cte_rms = math.sqrt(sum(value * value for value in self.cross_track_errors_m) / len(self.cross_track_errors_m)) if self.cross_track_errors_m else None
        heading_rms = math.sqrt(sum(value * value for value in self.heading_errors_rad) / len(self.heading_errors_rad)) if self.heading_errors_rad else None
        result: dict[str, object] = {
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
            "steering_saturation_fraction": self.steering_saturation_events / len(self.steering_commands_rad) if self.steering_commands_rad else 0.0,
            "invalid_command_events": self.invalid_command_events,
            "simulator_step_count": self.simulator_steps,
            "max_abs_steering_rad": max((abs(value) for value in self.steering_commands_rad), default=0.0),
            "cte_mean_m": sum(self.cross_track_errors_m) / len(self.cross_track_errors_m) if self.cross_track_errors_m else None,
            "cte_rms_m": cte_rms,
            "cte_p95_m": _percentile(absolute_cte, 0.95),
            "cte_max_m": max(absolute_cte) if absolute_cte else None,
            "heading_error_mean_rad": sum(self.heading_errors_rad) / len(self.heading_errors_rad) if self.heading_errors_rad else None,
            "heading_error_rms_rad": heading_rms,
            "heading_error_p95_rad": _percentile(absolute_heading, 0.95),
            "heading_error_max_rad": max(absolute_heading) if absolute_heading else None,
            "cte_pass_threshold": "UNSET",
            "terminal_state": terminal_state,
            "final_requested_command": final_requested_command,
            "final_applied_command": final_applied_command,
            "steps_after_terminal": steps_after_terminal,
            "requested_speed_min_mps": min(self.speed_commands_mps, default=None),
            "requested_speed_mean_mps": (
                sum(self.speed_commands_mps) / len(self.speed_commands_mps)
                if self.speed_commands_mps
                else None
            ),
            "requested_speed_max_mps": max(self.speed_commands_mps, default=None),
        }
        if metadata:
            result.update(metadata)
        return result
