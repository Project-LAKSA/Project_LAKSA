#!/usr/bin/env python3
"""Fail-closed host-session lease used by supervised batch trials.

The lease is deliberately a simple host-renewed file.  Its mtime is evaluated
with the Jetson wall clock, so loss of the Mac process, SSH transport, or its
renewal thread expires locally without trusting ROS time or a remote callback.
"""
from __future__ import annotations

import time
from pathlib import Path


class LeaseExpired(RuntimeError):
    pass


class FileLease:
    def __init__(self, path: Path, max_age_s: float, *, now=time.time):
        self.path = Path(path)
        self.max_age_s = float(max_age_s)
        self._now = now

    def age_s(self) -> float | None:
        try:
            return max(0.0, self._now() - self.path.stat().st_mtime)
        except FileNotFoundError:
            return None

    def valid(self) -> bool:
        age = self.age_s()
        return age is not None and age <= self.max_age_s

    def require(self) -> None:
        if not self.valid():
            raise LeaseExpired("SESSION_LEASE_EXPIRED")
