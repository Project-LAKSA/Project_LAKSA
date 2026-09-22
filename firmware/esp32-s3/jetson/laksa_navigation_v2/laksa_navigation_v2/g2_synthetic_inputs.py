"""Deterministic, test-only standard-message fixtures for G2.

This module is a trajectory and measurement generator, not an estimator. The
ROS publisher wrapper consumes these fixtures without changing their contract.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable

from .generate_contract_artifacts import derived_values, load_contract


SCENARIOS = (
    "G2_S001_STATIONARY", "G2_S002_STRAIGHT_FORWARD", "G2_S003_STRAIGHT_REVERSE", "G2_S004_LEFT_RADIUS", "G2_S005_RIGHT_RADIUS", "G2_S006_S_TURN", "G2_S007_STOP_START", "G2_S008_VIO_DROPOUT", "G2_S009_SPEED_DROPOUT", "G2_S010_ALL_INPUT_DROPOUT", "G2_S011_VIO_POSITION_OUTLIER", "G2_S012_YAW_OUTLIER", "G2_S013_DELAYED_SAMPLE", "G2_S014_OUT_OF_ORDER_SAMPLE", "G2_S015_SYNTHETIC_RAMP",
)
VIO_POSITION_STD_M = 0.015
VIO_YAW_STD_RAD = 0.012
SPEED_STD_MPS = 0.02
VY_CONSTRAINT_STD_MPS = SPEED_STD_MPS
DT_SEC = 0.05
SAMPLES = 80


@dataclass(frozen=True)
class Truth:
    stamp_sec: float
    x_m: float
    y_m: float
    yaw_rad: float
    vx_mps: float
    body_pitch_rad: float = 0.0


@dataclass(frozen=True)
class Measurement:
    truth: Truth
    vio_available: bool
    speed_available: bool
    vio_stamp_sec: float
    speed_stamp_sec: float
    vio_x_m: float
    vio_y_m: float
    vio_yaw_rad: float
    speed_mps: float


def diagonal_covariance(selected_variances: dict[int, float]) -> list[float]:
    """Return a six-by-six ROS covariance with no invented correlation."""
    covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        covariance[index] = 1.0
    for index, variance in selected_variances.items():
        covariance[index] = variance
    return covariance


def vio_pose_covariance() -> list[float]:
    return diagonal_covariance({0: VIO_POSITION_STD_M ** 2, 7: VIO_POSITION_STD_M ** 2, 35: VIO_YAW_STD_RAD ** 2})


def speed_twist_covariance() -> list[float]:
    return diagonal_covariance({0: SPEED_STD_MPS ** 2})


def vy_constraint_twist_covariance() -> list[float]:
    """Synthetic-only low-slip pseudo-measurement, never production default."""
    return diagonal_covariance({7: VY_CONSTRAINT_STD_MPS ** 2})


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _motion_for_case(case: str, index: int, values: dict) -> tuple[float, float]:
    t = index * DT_SEC
    if case == "G2_S002_STRAIGHT_FORWARD": return 0.25, 0.0
    if case == "G2_S003_STRAIGHT_REVERSE": return -0.20, 0.0
    if case == "G2_S004_LEFT_RADIUS": return 0.20, 0.20 / values["geometric_radius_left_m"]
    if case == "G2_S005_RIGHT_RADIUS": return 0.20, -0.20 / values["geometric_radius_right_m"]
    if case == "G2_S006_S_TURN":
        direction = 1.0 if index < SAMPLES // 2 else -1.0
        radius = values["geometric_radius_left_m"] if direction > 0 else values["geometric_radius_right_m"]
        return 0.18, direction * 0.18 / radius
    if case == "G2_S007_STOP_START": return (0.22 if 1.0 < t < 3.0 else 0.0), 0.0
    if case == "G2_S015_SYNTHETIC_RAMP": return 0.16, 0.0
    if case == "G2_S001_STATIONARY": return 0.0, 0.0
    return 0.18, 0.0


def generate_case(case: str, seed: int = 20260921) -> list[Measurement]:
    if case not in SCENARIOS:
        raise ValueError(f"unknown G2 scenario {case}")
    values = derived_values(load_contract())
    rng = random.Random(seed + SCENARIOS.index(case))
    x = y = yaw = 0.0
    samples: list[Measurement] = []
    for index in range(SAMPLES):
        vx, yaw_rate = _motion_for_case(case, index, values)
        if index:
            x += vx * math.cos(yaw) * DT_SEC
            y += vx * math.sin(yaw) * DT_SEC
            yaw = _wrap(yaw + yaw_rate * DT_SEC)
        truth = Truth(index * DT_SEC, x, y, yaw, vx, math.radians(8.0) * math.sin(index * DT_SEC) if case == "G2_S015_SYNTHETIC_RAMP" else 0.0)
        vio_available, speed_available = True, True
        if case == "G2_S008_VIO_DROPOUT" and index >= SAMPLES // 2: vio_available = False
        if case == "G2_S009_SPEED_DROPOUT" and index >= SAMPLES // 2: speed_available = False
        if case == "G2_S010_ALL_INPUT_DROPOUT" and index >= SAMPLES // 2: vio_available = speed_available = False
        vio_stamp, speed_stamp = truth.stamp_sec, truth.stamp_sec
        if case == "G2_S013_DELAYED_SAMPLE" and index == SAMPLES // 2: vio_stamp -= 0.20
        if case == "G2_S014_OUT_OF_ORDER_SAMPLE" and index == SAMPLES // 2: vio_stamp -= 0.10
        if case == "G2_S014_OUT_OF_ORDER_SAMPLE" and index == SAMPLES // 2 + 1:
            vio_stamp = samples[-1].vio_stamp_sec  # explicit duplicate timestamp after out-of-order input
        vio_x = x + rng.gauss(0.0, VIO_POSITION_STD_M)
        vio_y = y + rng.gauss(0.0, VIO_POSITION_STD_M)
        vio_yaw = _wrap(yaw + rng.gauss(0.0, VIO_YAW_STD_RAD))
        speed = vx + rng.gauss(0.0, SPEED_STD_MPS)
        if case == "G2_S011_VIO_POSITION_OUTLIER" and index == SAMPLES // 2: vio_x += 3.0
        if case == "G2_S012_YAW_OUTLIER" and index == SAMPLES // 2: vio_yaw = _wrap(vio_yaw + 2.5)
        samples.append(Measurement(truth, vio_available, speed_available, vio_stamp, speed_stamp, vio_x, vio_y, vio_yaw, speed))
    return samples


def all_cases() -> Iterable[tuple[str, list[Measurement]]]:
    return ((case, generate_case(case)) for case in SCENARIOS)
