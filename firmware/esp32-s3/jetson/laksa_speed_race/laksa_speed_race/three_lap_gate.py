"""Deterministic C1 three-lap mission gate; no driving intelligence lives here."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .c1_contract import MAX_LAPS


class MissionState(str, Enum):
    BOOT = "BOOT"
    WAIT_READY = "WAIT_READY"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


@dataclass
class ThreeLapGate:
    """Accept ordered lap events and fail closed on invalid C1 terminal events."""

    max_laps: int = MAX_LAPS
    state: MissionState = MissionState.BOOT
    lap_count: int = 0
    lap_times: list[float] = field(default_factory=list)
    done: bool = False
    fault: str | None = None
    terminal_zero_published: bool = False
    steps_after_terminal: int = 0

    def ready(self) -> None:
        if self.state is MissionState.BOOT:
            self.state = MissionState.WAIT_READY

    def start(self) -> None:
        if self.state is not MissionState.WAIT_READY:
            raise ValueError("mission cannot start before readiness")
        self.state = MissionState.RUNNING

    def fail(self, reason: str) -> None:
        if self.state not in (MissionState.COMPLETE, MissionState.FAULT):
            self.state = MissionState.FAULT
            self.done = True
            self.fault = reason

    def authorize_step(self) -> None:
        """Fail if a caller attempts to step outside the running state."""
        if self.state is not MissionState.RUNNING:
            self.steps_after_terminal += 1
            raise RuntimeError("simulator step requested outside RUNNING")

    def record_lap(self, lap_time_s: float) -> None:
        if self.state is not MissionState.RUNNING:
            self.fail("lap_event_outside_running")
            return
        if lap_time_s <= 0.0:
            self.fail("invalid_lap_time")
            return
        if self.lap_count >= self.max_laps:
            self.fail("lap_count_exceeded")
            return
        self.lap_count += 1
        self.lap_times.append(lap_time_s)
        if self.lap_count == self.max_laps:
            self.state = MissionState.STOPPING

    def stop_confirmed(self) -> None:
        if self.state is MissionState.STOPPING:
            self.state = MissionState.COMPLETE
            self.done = True
        elif self.state is MissionState.RUNNING:
            self.fail("simulator_done_before_three_laps")

    def record_terminal_zero(self, speed_mps: float, steering_rad: float) -> None:
        if speed_mps != 0.0 or steering_rad != 0.0:
            self.fail("terminal_command_not_zero")
            return
        self.terminal_zero_published = True
        self.stop_confirmed()

    @property
    def propulsion_permitted(self) -> bool:
        return self.state is MissionState.RUNNING
