#!/usr/bin/env python3
"""ROS-free safety contracts for the durable local characterization campaign.

This module intentionally has no publisher or subprocess side effects.  It is
shared by the local executor and the supervised stage runner so the condition
that permits motion is both testable and identical at every call site.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time
from pathlib import Path


TERMINAL = {"COMPLETE", "DATA_QUALITY_FAIL", "BLOCKED", "ABORT"}
LIFECYCLE = {"PENDING", "PREFLIGHT", "ARMED", "MOTION", "NEUTRALIZING", "ANALYZING", *TERMINAL}

# Every supervised maneuver needs the safety channels.  Moving trajectory
# tests additionally require actual fused-odometry messages; graph discovery
# alone is never sufficient evidence of a usable trajectory source.
BASE_REQUIRED_TOPICS = {
    "/joy": {"minimum_samples": 3, "minimum_rate_hz": 2.0},
    "/laksa/state": {"minimum_samples": 3, "minimum_rate_hz": 2.0},
    "/laksa/vesc/state": {"minimum_samples": 3, "minimum_rate_hz": 2.0},
    "/laksa/imu/data": {"minimum_samples": 8, "minimum_rate_hz": 8.0},
}
TRAJECTORY_TESTS = {"T21B", "T26"}
# These stages use the same mandatory sensor window, plus the signal that is
# actually used by their analyzers.  Merely finding a topic in the graph is
# intentionally insufficient.
SPEED_RESPONSE_TESTS = {"T23", "T24"}
MULTI_LEVEL_TESTS = {"T27"}


def required_topics(test_id: str) -> dict:
    result = {topic: dict(contract) for topic, contract in BASE_REQUIRED_TOPICS.items()}
    if test_id in TRAJECTORY_TESTS:
        result["/laksa/odometry/fused"] = {"minimum_samples": 10, "minimum_rate_hz": 8.0}
    if test_id in SPEED_RESPONSE_TESTS:
        result["/laksa/vesc/state"].update(minimum_samples=20, minimum_rate_hz=8.0)
        result["/laksa/command"] = {"minimum_samples": 10, "minimum_rate_hz": 4.0}
    if test_id in MULTI_LEVEL_TESTS:
        result["/laksa/vesc/state"].update(minimum_samples=30, minimum_rate_hz=8.0)
        result["/laksa/command"] = {"minimum_samples": 15, "minimum_rate_hz": 4.0}
    return result


def assess_observation(required: dict, observations: dict, *, freshness_s: float, now_ns: int | None = None) -> tuple[bool, list[str], dict]:
    """Validate a real observation window, not ROS graph endpoint names."""
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    details, failures = {}, []
    for topic, contract in required.items():
        samples = list(observations.get(topic, ()))
        count = len(samples)
        intervals = [(b - a) / 1e9 for a, b in zip(samples, samples[1:]) if b > a]
        rate = (1.0 / (sum(intervals) / len(intervals))) if intervals else 0.0
        age_s = None if not samples else (now_ns - samples[-1]) / 1e9
        finite = all(isinstance(value, int) and value > 0 for value in samples)
        details[topic] = {"sample_count": count, "sample_rate_hz": rate, "age_s": age_s,
                          "finite_timestamps": finite, "contract": contract}
        if count < contract["minimum_samples"]:
            failures.append("INSUFFICIENT_SAMPLES:" + topic)
        elif rate < contract["minimum_rate_hz"]:
            failures.append("LOW_RATE:" + topic)
        elif age_s is None or age_s > freshness_s:
            failures.append("STALE:" + topic)
        elif not finite:
            failures.append("INVALID_TIMESTAMP:" + topic)
    return not failures, failures, details


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


class CampaignState:
    """Single-file, atomic, restart-safe lifecycle record.

    The caller must write a terminal state on every exit path.  A stale active
    trial is converted to BLOCKED on the next invocation rather than becoming
    a permanent ``TRIAL_NOT_COMPLETE`` zombie.
    """
    def __init__(self, path: Path):
        self.path = Path(path)

    def load(self) -> dict:
        if not self.path.exists():
            return {"schema_version": "laksa-local-campaign-state-v1", "tests": {}, "active": None}
        value = json.loads(self.path.read_text())
        if value.get("schema_version") != "laksa-local-campaign-state-v1" or not isinstance(value.get("tests"), dict):
            raise RuntimeError("CORRUPT_OR_AMBIGUOUS_CAMPAIGN_STATE")
        active = value.get("active")
        if active:
            row = value["tests"].get(active, {})
            if row.get("state") not in TERMINAL:
                row.update(state="BLOCKED", terminal_reason="RECOVERED_STALE_ACTIVE_TRIAL", updated_monotonic_ns=time.monotonic_ns())
                value["tests"][active] = row
                value["active"] = None
                atomic_json(self.path, value)
        return value

    def transition(self, test_key: str, state: str, **extra) -> dict:
        if state not in LIFECYCLE:
            raise ValueError("INVALID_CAMPAIGN_LIFECYCLE_STATE:" + state)
        value = self.load()
        row = value["tests"].setdefault(test_key, {"state": "PENDING"})
        row.update(state=state, updated_monotonic_ns=time.monotonic_ns(), **extra)
        value["active"] = None if state in TERMINAL else test_key
        atomic_json(self.path, value)
        return value
