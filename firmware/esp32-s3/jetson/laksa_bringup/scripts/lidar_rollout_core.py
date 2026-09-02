#!/usr/bin/env python3

"""ROS-independent Ackermann rollout controller shared with simulation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class RolloutCommand:
    speed: float = 0.0
    steering: float = 0.0
    state: str = "STOPPED"


@dataclass(frozen=True)
class VehicleGeometry:
    wheelbase: float = 0.324
    max_left: float = 0.523
    max_right: float = 0.288
    footprint_circles: tuple[tuple[float, float], ...] = (
        (-0.14, 0.17),
        (0.10, 0.17),
        (0.32, 0.17),
    )


@dataclass(frozen=True)
class RolloutParameters:
    horizon_sec: float = 3.4
    clearance_weight: float = 3.5
    novelty_weight: float = 1.5
    continuity_weight: float = 0.30
    steering_weight: float = 0.20
    minimum_clearance: float = 0.20
    blocked_sec: float = 2.0
    reverse_sec: float = 1.80
    forward_recovery_sec: float = 2.50
    reverse_distance_m: float = 0.36
    forward_recovery_yaw_rad: float = 0.75
    recovery_phase_timeout_sec: float = 5.0
    cooldown_sec: float = 10.0
    forward_speed: float = 0.2414
    reverse_speed: float = 0.2173


class VisitMemory:
    def __init__(self, resolution: float = 0.20) -> None:
        self.resolution = resolution
        self.visited: dict[tuple[int, int], int] = {}

    def reset(self) -> None:
        self.visited.clear()

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def mark(self, pose) -> None:
        key = self._key(pose.x, pose.y)
        self.visited[key] = self.visited.get(key, 0) + 1

    def cost(self, x: float, y: float) -> float:
        ix, iy = self._key(x, y)
        return sum(
            self.visited.get((ix + dx, iy + dy), 0)
            for dx in range(-1, 2)
            for dy in range(-1, 2)
        )


def bicycle_step(pose, speed: float, steering: float, dt: float,
                 geometry: VehicleGeometry) -> Pose2D:
    steering = clamp(steering, -geometry.max_right, geometry.max_left)
    yaw_rate = speed * math.tan(steering) / geometry.wheelbase
    midpoint = pose.yaw + 0.5 * yaw_rate * dt
    yaw = (pose.yaw + yaw_rate * dt + math.pi) % (2.0 * math.pi) - math.pi
    return Pose2D(
        pose.x + speed * math.cos(midpoint) * dt,
        pose.y + speed * math.sin(midpoint) * dt,
        yaw,
    )


class AckermannRolloutController:
    """Score collision-free forward arcs and run a bounded K-turn if blocked."""

    name = "ackermann_rollout"

    def __init__(self, params: RolloutParameters | None = None,
                 geometry: VehicleGeometry | None = None) -> None:
        self.params = params or RolloutParameters()
        self.geometry = geometry or VehicleGeometry()
        self.reset()

    def reset(self) -> None:
        self.previous_steering = 0.0
        self.blocked_since: float | None = None
        self.reverse_until = 0.0
        self.forward_recovery_until = 0.0
        self.cooldown_until = 0.0
        self.recovery_turn = 1.0
        self.recovery_phases = 0
        self.recovery_phase_start: Pose2D | None = None

    @staticmethod
    def _distance(first, second) -> float:
        return math.hypot(first.x - second.x, first.y - second.y)

    @staticmethod
    def _yaw_change(first, second) -> float:
        return abs((second.yaw - first.yaw + math.pi) % (2.0 * math.pi) - math.pi)

    def _start_reverse(self, pose, now: float) -> None:
        self.reverse_until = now + self.params.recovery_phase_timeout_sec
        self.recovery_phase_start = Pose2D(pose.x, pose.y, pose.yaw)

    def _start_forward_recovery(self, pose, now: float) -> None:
        self.reverse_until = 0.0
        self.forward_recovery_until = now + self.params.recovery_phase_timeout_sec
        self.recovery_phase_start = Pose2D(pose.x, pose.y, pose.yaw)

    def _simulate(self, pose, speed: float, steering: float,
                  duration: float, dt: float = 0.30) -> list[Pose2D]:
        result = []
        state = Pose2D(pose.x, pose.y, pose.yaw)
        for _ in range(max(1, int(duration / dt))):
            state = bicycle_step(state, speed, steering, dt, self.geometry)
            result.append(state)
        return result

    def _scan_clearance(self, pose, trajectory: Iterable[Pose2D], scan) -> float:
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        obstacles = [
            (pose.x + cosine * x - sine * y, pose.y + sine * x + cosine * y)
            for x, y in scan.points_base[::2]
        ]
        if not obstacles:
            return 5.0
        clearance = 5.0
        for state in trajectory:
            for offset, radius in self.geometry.footprint_circles:
                cx = state.x + math.cos(state.yaw) * offset
                cy = state.y + math.sin(state.yaw) * offset
                nearest_sq = min(
                    (cx - ox) * (cx - ox) + (cy - oy) * (cy - oy)
                    for ox, oy in obstacles
                )
                clearance = min(clearance, math.sqrt(nearest_sq) - radius)
                if clearance < 0.05:
                    return clearance
        return clearance

    def command(self, pose, scan, memory, now: float) -> RolloutCommand:
        p = self.params
        rear = scan.sector(math.pi, math.radians(25), 0.10)
        if self.reverse_until > 0.0 and now >= self.reverse_until:
            self._start_forward_recovery(pose, now)
        if now < self.reverse_until:
            if rear < 0.45:
                self.reverse_until = now
                return RolloutCommand(0.0, 0.0, "RECOVERY_REAR_BLOCKED")
            if (
                self.recovery_phase_start is not None
                and self._distance(self.recovery_phase_start, pose)
                >= p.reverse_distance_m
            ):
                self._start_forward_recovery(pose, now)
                return RolloutCommand(
                    p.forward_speed,
                    self.recovery_turn * 0.27,
                    "RECOVERING_FORWARD_TURN",
                )
            return RolloutCommand(
                -p.reverse_speed,
                -self.recovery_turn * 0.27,
                "RECOVERING_REVERSE",
            )
        if now < self.forward_recovery_until:
            front = scan.sector(0.0, math.radians(18), 0.10)
            if front < 0.42:
                if rear >= 0.55 and self.recovery_phases < 3:
                    self.recovery_phases += 1
                    self._start_reverse(pose, now)
                    self.forward_recovery_until = 0.0
                    return RolloutCommand(
                        -p.reverse_speed,
                        -self.recovery_turn * 0.27,
                        "RECOVERING_REVERSE",
                    )
                return RolloutCommand(0.0, 0.0, "RECOVERY_FRONT_BLOCKED")
            if (
                self.recovery_phase_start is not None
                and self._yaw_change(self.recovery_phase_start, pose)
                >= p.forward_recovery_yaw_rad
            ):
                self.forward_recovery_until = 0.0
                self.cooldown_until = now + p.cooldown_sec
                self.recovery_phases = 0
                self.recovery_phase_start = None
            return RolloutCommand(
                p.forward_speed,
                self.recovery_turn * 0.27,
                "RECOVERING_FORWARD_TURN",
            )
        if self.forward_recovery_until > 0.0:
            self.forward_recovery_until = 0.0
            self.cooldown_until = now + p.cooldown_sec
            self.recovery_phases = 0

        front_left = scan.sector(math.radians(40), math.radians(28), 0.20)
        front_right = scan.sector(math.radians(-40), math.radians(28), 0.20)
        self.recovery_turn = 1.0 if front_left >= front_right else -1.0
        candidates = (
            -self.geometry.max_right,
            -0.18,
            -0.08,
            0.0,
            0.13,
            0.30,
            self.geometry.max_left,
        )
        best: tuple[float, float] | None = None
        for steering in candidates:
            trajectory = self._simulate(
                pose, p.forward_speed, steering, p.horizon_sec
            )
            clearance = self._scan_clearance(pose, trajectory, scan)
            if clearance < p.minimum_clearance:
                continue
            endpoint = trajectory[-1]
            novelty = 1.0 / (1.0 + memory.cost(endpoint.x, endpoint.y))
            score = (
                p.clearance_weight * min(clearance, 1.0)
                - p.continuity_weight * abs(steering - self.previous_steering)
                - p.steering_weight * abs(steering)
                + p.novelty_weight * novelty
            )
            if best is None or score > best[0]:
                best = score, steering
        if best is not None:
            self.blocked_since = None
            self.previous_steering = best[1]
            return RolloutCommand(p.forward_speed, best[1], "FORWARD_ROLLOUT")

        self.blocked_since = self.blocked_since or now
        if (
            now - self.blocked_since >= p.blocked_sec
            and rear >= 0.70
            and now >= self.cooldown_until
        ):
            self.recovery_phases = 1
            self._start_reverse(pose, now)
            return RolloutCommand(
                -p.reverse_speed,
                -self.recovery_turn * 0.27,
                "RECOVERING_REVERSE",
            )
        return RolloutCommand(0.0, 0.0, "BLOCKED_WAITING")
