#!/usr/bin/env python3

"""Dependency-free Ackermann/LiDAR exploration simulation for LAKSA.

The simulator is deliberately deterministic and fast enough for parameter
sweeps.  It models the measured asymmetric steering, the Slash footprint,
rear-axle bicycle kinematics, a forward-mounted 360-degree LiDAR, collision,
visited-space memory, and observable-area coverage.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

SCRIPTS = Path(__file__).resolve().parents[1] / "laksa_bringup" / "scripts"
sys.path.insert(0, str(SCRIPTS))
from lidar_rollout_core import AckermannRolloutController  # noqa: E402


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class Command:
    speed: float = 0.0
    steering: float = 0.0
    state: str = "STOPPED"


@dataclass
class Vehicle:
    wheelbase: float = 0.324
    length: float = 0.568
    width: float = 0.296
    max_left: float = 0.523
    max_right: float = 0.288
    lidar_x: float = 0.315
    forward_speed: float = 0.2414
    reverse_speed: float = 0.2173

    @property
    def footprint_circles(self) -> tuple[tuple[float, float], ...]:
        # Longitudinal offsets from the rear axle and conservative radii.
        return ((-0.14, 0.17), (0.10, 0.17), (0.32, 0.17))


class GridWorld:
    def __init__(self, width_m: float, height_m: float, resolution: float = 0.05):
        self.resolution = resolution
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self.occupied = bytearray(self.width * self.height)
        self._rectangle(0.0, 0.0, width_m, resolution)
        self._rectangle(0.0, height_m - resolution, width_m, height_m)
        self._rectangle(0.0, 0.0, resolution, height_m)
        self._rectangle(width_m - resolution, 0.0, width_m, height_m)

    def index(self, ix: int, iy: int) -> int:
        return iy * self.width + ix

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def center(self, ix: int, iy: int) -> tuple[float, float]:
        return ((ix + 0.5) * self.resolution, (iy + 0.5) * self.resolution)

    def is_occupied(self, x: float, y: float) -> bool:
        ix, iy = self.cell(x, y)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return True
        return bool(self.occupied[self.index(ix, iy)])

    def _rectangle(self, x0: float, y0: float, x1: float, y1: float) -> None:
        ix0, iy0 = self.cell(max(0.0, x0), max(0.0, y0))
        ix1 = min(self.width, int(math.ceil(x1 / self.resolution)))
        iy1 = min(self.height, int(math.ceil(y1 / self.resolution)))
        for iy in range(iy0, iy1):
            for ix in range(ix0, ix1):
                self.occupied[self.index(ix, iy)] = 1

    def add_rectangle(self, x0: float, y0: float, x1: float, y1: float) -> None:
        self._rectangle(x0, y0, x1, y1)

    def raycast(self, x: float, y: float, angle: float, maximum: float) -> float:
        step = self.resolution * 0.60
        distance = 0.0
        cosine, sine = math.cos(angle), math.sin(angle)
        while distance <= maximum:
            if self.is_occupied(x + cosine * distance, y + sine * distance):
                return distance
            distance += step
        return maximum

    def collision(self, pose: Pose, vehicle: Vehicle) -> bool:
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        sample_step = self.resolution * 0.75
        for offset, radius in vehicle.footprint_circles:
            cx = pose.x + cosine * offset
            cy = pose.y + sine * offset
            rings = max(8, int(2.0 * math.pi * radius / sample_step))
            if self.is_occupied(cx, cy):
                return True
            for ring in (0.55, 1.0):
                for index in range(rings):
                    angle = 2.0 * math.pi * index / rings
                    if self.is_occupied(
                        cx + math.cos(angle) * radius * ring,
                        cy + math.sin(angle) * radius * ring,
                    ):
                        return True
        return False


def kitchen_world(name: str) -> tuple[GridWorld, list[Pose]]:
    world = GridWorld(7.5, 5.8)
    if name == "island":
        world.add_rectangle(0.05, 4.75, 5.35, 5.75)  # rear counter
        world.add_rectangle(0.05, 0.05, 0.92, 4.05)  # left cabinets
        world.add_rectangle(6.55, 0.05, 7.45, 2.55)  # right cabinets
        world.add_rectangle(3.05, 2.10, 4.55, 3.18)  # island
        starts = [Pose(1.65, 0.90, 0.0), Pose(5.65, 4.15, math.pi)]
    elif name == "corner":
        # An L-shaped route wide enough for the measured 1.09 m right radius.
        world.add_rectangle(0.05, 0.05, 0.90, 4.70)
        world.add_rectangle(2.65, 0.05, 7.45, 2.00)
        world.add_rectangle(4.55, 3.55, 7.45, 5.75)
        starts = [Pose(1.75, 0.85, math.pi / 2.0), Pose(3.55, 2.80, 0.0)]
    elif name == "bottleneck":
        world.add_rectangle(0.05, 4.85, 7.45, 5.75)
        world.add_rectangle(0.05, 0.05, 1.00, 3.90)
        world.add_rectangle(3.10, 1.75, 4.35, 3.85)
        world.add_rectangle(6.35, 0.05, 7.45, 3.20)
        starts = [Pose(1.70, 0.85, 0.0), Pose(5.50, 4.15, math.pi)]
    else:
        raise ValueError(f"Unknown scenario: {name}")
    return world, starts


@dataclass
class Scan:
    ranges: list[float]
    angles: list[float]
    points_base: list[tuple[float, float]]

    def sector(self, center: float, half_width: float, fraction: float) -> float:
        values = sorted(
            distance
            for distance, angle in zip(self.ranges, self.angles)
            if abs(wrap(angle - center)) <= half_width
        )
        if not values:
            return 5.0
        return values[int(clamp(fraction, 0.0, 1.0) * (len(values) - 1))]


@dataclass
class Memory:
    resolution: float = 0.20
    visited: dict[tuple[int, int], int] = field(default_factory=dict)

    def key(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def mark(self, pose: Pose) -> None:
        key = self.key(pose.x, pose.y)
        self.visited[key] = self.visited.get(key, 0) + 1

    def cost(self, x: float, y: float) -> float:
        ix, iy = self.key(x, y)
        return sum(
            self.visited.get((ix + dx, iy + dy), 0)
            for dx in range(-1, 2)
            for dy in range(-1, 2)
        )


class Controller:
    name = "base"

    def reset(self) -> None:
        pass

    def command(self, pose: Pose, scan: Scan, memory: Memory, now: float) -> Command:
        raise NotImplementedError


class SectorController(Controller):
    name = "sector_centering"

    def __init__(self) -> None:
        self.filtered = 0.0
        self.blocked_since: float | None = None
        self.reverse_until = 0.0
        self.cooldown_until = 0.0
        self.turn = 1.0

    def reset(self) -> None:
        self.__init__()

    def command(self, pose: Pose, scan: Scan, memory: Memory, now: float) -> Command:
        front = scan.sector(0.0, math.radians(18), 0.10)
        rear = scan.sector(math.pi, math.radians(22), 0.10)
        left = scan.sector(math.radians(78), math.radians(24), 0.50)
        right = scan.sector(math.radians(-78), math.radians(24), 0.50)
        front_left = scan.sector(math.radians(42), math.radians(23), 0.20)
        front_right = scan.sector(math.radians(-42), math.radians(23), 0.20)
        opening = clamp(front_left - front_right, -1.0, 1.0)
        if abs(opening) > 0.08:
            self.turn = math.copysign(1.0, opening)
        if now < self.reverse_until:
            return Command(-0.2173, -self.turn * 0.27, "RECOVERING_REVERSE")
        if front <= 0.42:
            self.blocked_since = self.blocked_since or now
            if now - self.blocked_since >= 2.0 and rear >= 0.70 and now >= self.cooldown_until:
                self.reverse_until = now + 0.75
                self.cooldown_until = now + 8.0
                return Command(-0.2173, -self.turn * 0.27, "RECOVERING_REVERSE")
            return Command(0.0, 0.0, "BLOCKED_WAITING")
        self.blocked_since = None
        steering = 0.65 * clamp(left - right, -0.8, 0.8)
        if front < 0.95:
            proximity = 1.0 - (front - 0.42) / (0.95 - 0.42)
            steering += 0.42 * opening + self.turn * 0.27 * proximity
        else:
            steering += 0.084 * opening
        self.filtered += 0.30 * (steering - self.filtered)
        return Command(0.2414, clamp(self.filtered, -0.27, 0.27), "FORWARD")


@dataclass(frozen=True)
class RolloutParameters:
    horizon_sec: float = 2.6
    clearance_weight: float = 5.0
    novelty_weight: float = 2.5
    continuity_weight: float = 0.30
    steering_weight: float = 0.20
    minimum_clearance: float = 0.20
    blocked_sec: float = 2.0
    reverse_sec: float = 1.80
    forward_recovery_sec: float = 2.50
    cooldown_sec: float = 10.0
    forward_speed: float = 0.2414
    reverse_speed: float = 0.2173


class RolloutController(Controller):
    name = "ackermann_rollout"

    def __init__(self, params: RolloutParameters):
        self.params = params
        self.previous_steering = 0.0
        self.blocked_since: float | None = None
        self.reverse_until = 0.0
        self.forward_recovery_until = 0.0
        self.cooldown_until = 0.0
        self.recovery_turn = 1.0
        self.recovery_phases = 0

    def reset(self) -> None:
        self.__init__(self.params)

    @staticmethod
    def _simulate(pose: Pose, speed: float, steering: float, duration: float,
                  dt: float = 0.30) -> list[Pose]:
        result = []
        state = Pose(pose.x, pose.y, pose.yaw)
        steps = max(1, int(duration / dt))
        for _ in range(steps):
            state = bicycle_step(state, speed, steering, dt, Vehicle())
            result.append(state)
        return result

    @staticmethod
    def _scan_clearance(pose: Pose, trajectory: Iterable[Pose], scan: Scan) -> float:
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        obstacles = [
            (pose.x + cosine * x - sine * y, pose.y + sine * x + cosine * y)
            for x, y in scan.points_base[::2]
        ]
        if not obstacles:
            return 5.0
        clearance = 5.0
        for state in trajectory:
            for offset, radius in Vehicle().footprint_circles:
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

    def command(self, pose: Pose, scan: Scan, memory: Memory, now: float) -> Command:
        rear = scan.sector(math.pi, math.radians(25), 0.10)
        if now < self.reverse_until:
            if rear < 0.45:
                self.reverse_until = now
                return Command(0.0, 0.0, "RECOVERY_REAR_BLOCKED")
            return Command(-0.2173, -self.recovery_turn * 0.27, "RECOVERING_REVERSE")
        if now < self.forward_recovery_until:
            front = scan.sector(0.0, math.radians(18), 0.10)
            if front < 0.42:
                if rear >= 0.55 and self.recovery_phases < 3:
                    self.recovery_phases += 1
                    self.reverse_until = now + self.params.reverse_sec
                    self.forward_recovery_until = (
                        self.reverse_until + self.params.forward_recovery_sec
                    )
                    return Command(
                        -0.2173,
                        -self.recovery_turn * 0.27,
                        "RECOVERING_REVERSE",
                    )
                return Command(0.0, 0.0, "RECOVERY_FRONT_BLOCKED")
            return Command(0.2414, self.recovery_turn * 0.27, "RECOVERING_FORWARD_TURN")
        if self.forward_recovery_until > 0.0:
            # A complete reverse/forward sequence has ended. Only now impose
            # the cooldown; an unfinished K-turn may chain up to three phases.
            self.forward_recovery_until = 0.0
            self.cooldown_until = now + self.params.cooldown_sec
            self.recovery_phases = 0

        candidates = (-0.288, -0.18, -0.08, 0.0, 0.13, 0.30, 0.523)
        best: tuple[float, float] | None = None
        front_left = scan.sector(math.radians(40), math.radians(28), 0.20)
        front_right = scan.sector(math.radians(-40), math.radians(28), 0.20)
        self.recovery_turn = 1.0 if front_left >= front_right else -1.0
        for steering in candidates:
            trajectory = self._simulate(
                pose, 0.2414, steering, self.params.horizon_sec
            )
            clearance = self._scan_clearance(pose, trajectory, scan)
            if clearance < self.params.minimum_clearance:
                continue
            endpoint = trajectory[-1]
            novelty = 1.0 / (1.0 + memory.cost(endpoint.x, endpoint.y))
            continuity = -abs(steering - self.previous_steering)
            steering_cost = -abs(steering)
            score = (
                self.params.clearance_weight * min(clearance, 1.0)
                + self.params.novelty_weight * novelty
                + self.params.continuity_weight * continuity
                + self.params.steering_weight * steering_cost
            )
            if best is None or score > best[0]:
                best = score, steering

        if best is not None:
            self.blocked_since = None
            self.previous_steering = best[1]
            return Command(0.2414, best[1], "FORWARD_ROLLOUT")

        self.blocked_since = self.blocked_since or now
        if (
            now - self.blocked_since >= self.params.blocked_sec
            and rear >= 0.70
            and now >= self.cooldown_until
        ):
            self.reverse_until = now + self.params.reverse_sec
            self.recovery_phases = 1
            self.forward_recovery_until = (
                self.reverse_until + self.params.forward_recovery_sec
            )
            self.cooldown_until = now + self.params.cooldown_sec
            return Command(-0.2173, -self.recovery_turn * 0.27, "RECOVERING_REVERSE")
        return Command(0.0, 0.0, "BLOCKED_WAITING")


def bicycle_step(pose: Pose, speed: float, steering: float, dt: float,
                 vehicle: Vehicle) -> Pose:
    steering = clamp(steering, -vehicle.max_right, vehicle.max_left)
    yaw_rate = speed * math.tan(steering) / vehicle.wheelbase
    midpoint = pose.yaw + 0.5 * yaw_rate * dt
    return Pose(
        pose.x + speed * math.cos(midpoint) * dt,
        pose.y + speed * math.sin(midpoint) * dt,
        wrap(pose.yaw + yaw_rate * dt),
    )


def lidar_scan(world: GridWorld, pose: Pose, vehicle: Vehicle, beams: int,
               maximum: float, observed: bytearray,
               rng: random.Random | None = None,
               noise_std: float = 0.0) -> Scan:
    lidar_x = pose.x + math.cos(pose.yaw) * vehicle.lidar_x
    lidar_y = pose.y + math.sin(pose.yaw) * vehicle.lidar_x
    ranges, angles, points = [], [], []
    for index in range(beams):
        relative = -math.pi + 2.0 * math.pi * index / beams
        distance = world.raycast(lidar_x, lidar_y, pose.yaw + relative, maximum)
        if rng is not None and noise_std > 0.0:
            distance = clamp(distance + rng.gauss(0.0, noise_std), 0.05, maximum)
        ranges.append(distance)
        angles.append(relative)
        if distance < maximum - world.resolution:
            points.append((
                vehicle.lidar_x + math.cos(relative) * distance,
                math.sin(relative) * distance,
            ))
        ray_step = world.resolution * 2.0
        sample = 0.0
        while sample < distance:
            x = lidar_x + math.cos(pose.yaw + relative) * sample
            y = lidar_y + math.sin(pose.yaw + relative) * sample
            ix, iy = world.cell(x, y)
            if 0 <= ix < world.width and 0 <= iy < world.height:
                observed[world.index(ix, iy)] = 1
            sample += ray_step
    return Scan(ranges, angles, points)


@dataclass
class Result:
    controller: str
    scenario: str
    start: int
    coverage: float
    collisions: int
    blocked_seconds: float
    reverse_meters: float
    recoveries: int
    distance: float
    final_state: str
    elapsed_seconds: float
    reached_coverage_target: bool
    path: list[tuple[float, float]] = field(repr=False)

    def json(self) -> dict:
        data = vars(self).copy()
        data.pop("path")
        return data


def observable_free_cells(world: GridWorld, vehicle: Vehicle) -> bytearray:
    # Conservative circle inflation approximates the three-circle footprint.
    result = bytearray(world.width * world.height)
    radius_cells = int(math.ceil(0.19 / world.resolution))
    occupied_cells = [
        (index % world.width, index // world.width)
        for index, value in enumerate(world.occupied) if value
    ]
    inflated = bytearray(world.occupied)
    for ox, oy in occupied_cells:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                ix, iy = ox + dx, oy + dy
                if 0 <= ix < world.width and 0 <= iy < world.height:
                    inflated[world.index(ix, iy)] = 1
    for index, value in enumerate(inflated):
        result[index] = 0 if value else 1
    return result


def simulate(controller: Controller, scenario: str, start_index: int,
             duration: float = 150.0, dt: float = 0.10,
             coverage_target: float = 0.98, seed: int = 0,
             sensor_noise_std: float = 0.0,
             command_latency_sec: float = 0.0,
             steering_time_constant_sec: float = 0.0,
             odom_position_noise_std: float = 0.0,
             odom_yaw_noise_std: float = 0.0) -> Result:
    world, starts = kitchen_world(scenario)
    vehicle = Vehicle()
    pose = starts[start_index]
    if world.collision(pose, vehicle):
        raise RuntimeError(f"Invalid colliding start in {scenario}:{start_index}")
    controller.reset()
    memory = Memory()
    observed = bytearray(world.width * world.height)
    observable = observable_free_cells(world, vehicle)
    path = [(pose.x, pose.y)]
    collisions = 0
    blocked_time = 0.0
    reverse_meters = 0.0
    recoveries = 0
    distance = 0.0
    previous_state = ""
    last_command = Command()
    applied_command = Command()
    command_queue: list[tuple[float, Command]] = []
    rng = random.Random(seed)
    target_since: float | None = None
    elapsed = 0.0
    reached_target = False
    observable_total = max(1, sum(observable))
    for step in range(int(duration / dt)):
        now = step * dt
        elapsed = now + dt
        scan = lidar_scan(
            world, pose, vehicle, 48, 5.0, observed, rng, sensor_noise_std
        )
        sensed_pose = Pose(
            pose.x + rng.gauss(0.0, odom_position_noise_std),
            pose.y + rng.gauss(0.0, odom_position_noise_std),
            wrap(pose.yaw + rng.gauss(0.0, odom_yaw_noise_std)),
        )
        memory.mark(sensed_pose)
        command = controller.command(sensed_pose, scan, memory, now)
        command_queue.append((now + command_latency_sec, command))
        while command_queue and command_queue[0][0] <= now:
            _, requested = command_queue.pop(0)
            applied_command.speed = requested.speed
            applied_command.state = requested.state
            if steering_time_constant_sec <= 0.0:
                applied_command.steering = requested.steering
            else:
                alpha = clamp(dt / steering_time_constant_sec, 0.0, 1.0)
                applied_command.steering += alpha * (
                    requested.steering - applied_command.steering
                )
        command = Command(
            applied_command.speed,
            applied_command.steering,
            applied_command.state,
        )
        if command.state == "BLOCKED_WAITING":
            blocked_time += dt
        if command.state == "RECOVERING_REVERSE":
            reverse_meters += abs(command.speed) * dt
            if previous_state != command.state:
                recoveries += 1
        candidate = bicycle_step(pose, command.speed, command.steering, dt, vehicle)
        if world.collision(candidate, vehicle):
            collisions += 1
            command = Command(0.0, 0.0, "COLLISION_STOP")
        else:
            pose = candidate
            distance += abs(command.speed) * dt
        if step % 5 == 0:
            path.append((pose.x, pose.y))
        previous_state = command.state
        last_command = command
        if step % 10 == 0:
            known_now = sum(
                1 for index, allowed in enumerate(observable)
                if allowed and observed[index]
            )
            if known_now / observable_total >= coverage_target:
                target_since = target_since if target_since is not None else now
                if now - target_since >= 3.0:
                    reached_target = True
                    last_command = Command(0.0, 0.0, "COVERAGE_TARGET_REACHED")
                    break
            else:
                target_since = None
    known = sum(1 for index, allowed in enumerate(observable) if allowed and observed[index])
    total = observable_total
    return Result(
        controller.name,
        scenario,
        start_index,
        known / total,
        collisions,
        blocked_time,
        reverse_meters,
        recoveries,
        distance,
        last_command.state,
        elapsed,
        reached_target,
        path,
    )


def aggregate(results: list[Result]) -> dict:
    count = len(results)
    return {
        "runs": count,
        "mean_coverage": sum(r.coverage for r in results) / count,
        "minimum_coverage": min(r.coverage for r in results),
        "collisions": sum(r.collisions for r in results),
        "blocked_seconds": sum(r.blocked_seconds for r in results),
        "reverse_meters": sum(r.reverse_meters for r in results),
        "recoveries": sum(r.recoveries for r in results),
        "distance_meters": sum(r.distance for r in results),
        "targets_reached": sum(r.reached_coverage_target for r in results),
        "mean_elapsed_seconds": sum(r.elapsed_seconds for r in results) / count,
    }


def svg_report(path: Path, results: list[Result]) -> None:
    scale, margin = 100.0, 20.0
    panel_w, panel_h = 750.0, 580.0
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{panel_w * len(results):.0f}" height="{panel_h + 70:.0f}">',
        '<rect width="100%" height="100%" fill="#09111f"/>',
    ]
    colors = {"sector_centering": "#ffb84d", "ackermann_rollout": "#33d6ff"}
    for panel, result in enumerate(results):
        world, _ = kitchen_world(result.scenario)
        offset = panel * panel_w
        pieces.append(f'<g transform="translate({offset + margin},{margin})">')
        pieces.append(f'<rect width="{world.width * world.resolution * scale}" height="{world.height * world.resolution * scale}" fill="#e5e7eb"/>')
        for index, occupied in enumerate(world.occupied):
            if not occupied:
                continue
            ix, iy = index % world.width, index // world.width
            pieces.append(
                f'<rect x="{ix * world.resolution * scale:.1f}" y="{(world.height - iy - 1) * world.resolution * scale:.1f}" width="{world.resolution * scale + 0.2:.1f}" height="{world.resolution * scale + 0.2:.1f}" fill="#252b35"/>'
            )
        points = " ".join(
            f"{x * scale:.1f},{(world.height * world.resolution - y) * scale:.1f}"
            for x, y in result.path
        )
        pieces.append(f'<polyline points="{points}" fill="none" stroke="{colors.get(result.controller, "#fff")}" stroke-width="3"/>')
        pieces.append('</g>')
        pieces.append(
            f'<text x="{offset + margin}" y="{panel_h + 30}" fill="#e8f0ff" font-family="sans-serif" font-size="16">{result.controller} / {result.scenario} / coverage {result.coverage:.1%} / collisions {result.collisions}</text>'
        )
    pieces.append('</svg>')
    path.write_text("\n".join(pieces), encoding="utf-8")


def parameter_candidates() -> Iterable[RolloutParameters]:
    for horizon in (2.0, 2.8, 3.4):
        for clearance in (3.5, 6.0):
            for novelty in (1.5, 2.5, 4.0):
                yield RolloutParameters(
                    horizon_sec=horizon,
                    clearance_weight=clearance,
                    novelty_weight=novelty,
                )


def tune(duration: float) -> tuple[RolloutParameters, list[dict]]:
    leaderboard = []
    scenarios = ("island", "corner", "bottleneck")
    for params in parameter_candidates():
        results = [
            simulate(AckermannRolloutController(params), scenario, 0, duration)
            for scenario in scenarios
        ]
        metrics = aggregate(results)
        # Collisions dominate, then minimum coverage, then unnecessary reverse.
        score = (
            metrics["mean_coverage"] * 100.0
            + metrics["minimum_coverage"] * 50.0
            - metrics["collisions"] * 25.0
            - metrics["blocked_seconds"] * 0.05
            - metrics["reverse_meters"] * 2.0
        )
        leaderboard.append({"score": score, "params": vars(params), **metrics})
    leaderboard.sort(key=lambda item: item["score"], reverse=True)
    return RolloutParameters(**leaderboard[0]["params"]), leaderboard


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=150.0)
    parser.add_argument("--tune-duration", type=float, default=90.0)
    parser.add_argument("--output", type=Path, default=Path("/tmp/laksa-simulation"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    best, leaderboard = tune(args.tune_duration)
    all_results = []
    for scenario in ("island", "corner", "bottleneck"):
        for start in (0, 1):
            all_results.append(simulate(SectorController(), scenario, start, args.duration))
            all_results.append(
                simulate(AckermannRolloutController(best), scenario, start, args.duration)
            )
    by_controller = {}
    for name in ("sector_centering", "ackermann_rollout"):
        by_controller[name] = aggregate([r for r in all_results if r.controller == name])
    report = {
        "vehicle": vars(Vehicle()),
        "runtime_core": "lidar_rollout_core.AckermannRolloutController",
        "best_rollout_parameters": vars(best),
        "aggregate": by_controller,
        "runs": [result.json() for result in all_results],
        "leaderboard_top_5": leaderboard[:5],
    }
    (args.output / "report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    # Plot one representative baseline/best pair without third-party graphics.
    pair = [
        next(r for r in all_results if r.controller == "sector_centering" and r.scenario == "island" and r.start == 0),
        next(r for r in all_results if r.controller == "ackermann_rollout" and r.scenario == "island" and r.start == 0),
    ]
    svg_report(args.output / "island_comparison.svg", pair)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
