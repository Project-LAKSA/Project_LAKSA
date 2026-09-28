"""Bounded ownership and cleanup for an isolated subprocess session.

The caller remains outside the child session.  Every descendant that does not
explicitly detach therefore shares the session leader's process group and can
be stopped without matching process names or touching unrelated ROS nodes.
"""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import IO, Mapping, Sequence


PR_SET_CHILD_SUBREAPER = 36


@dataclass(frozen=True)
class ProcessRecord:
    pid: int
    ppid: int
    pgid: int
    sid: int
    state: str
    command: str


@dataclass(frozen=True)
class ShutdownResult:
    success: bool
    phase: str
    pgid: int
    observed_pids: tuple[int, ...]
    remaining_pids: tuple[int, ...]
    leader_returncode: int | None


def _enable_child_subreaper() -> bool:
    """Make orphaned descendants reapable by this harness on Linux."""

    if not sys.platform.startswith("linux"):
        return False
    libc = ctypes.CDLL(None, use_errno=True)
    result = libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0)
    if result != 0:
        errno = ctypes.get_errno()
        raise OSError(errno, os.strerror(errno))
    return True


def _linux_process_records() -> list[ProcessRecord]:
    records: list[ProcessRecord] = []
    proc = Path("/proc")
    if not proc.is_dir():
        return records
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
            close = stat.rfind(")")
            command = stat[stat.find("(") + 1 : close]
            fields = stat[close + 2 :].split()
            records.append(
                ProcessRecord(
                    pid=int(entry.name),
                    state=fields[0],
                    ppid=int(fields[1]),
                    pgid=int(fields[2]),
                    sid=int(fields[3]),
                    command=command,
                )
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return records


def process_group_records(pgid: int) -> list[ProcessRecord]:
    """Return every still-present process in ``pgid`` on Linux."""

    return [record for record in _linux_process_records() if record.pgid == pgid]


class OwnedProcessSession:
    """Own one child session and shut down all descendants with bounded waits."""

    def __init__(
        self,
        command: Sequence[str],
        *,
        env: Mapping[str, str] | None = None,
        cwd: str | os.PathLike[str] | None = None,
        stdout: IO[bytes] | int | None = None,
        stderr: IO[bytes] | int | None = None,
    ) -> None:
        if not command:
            raise ValueError("command must not be empty")
        _enable_child_subreaper()
        self.process = subprocess.Popen(
            list(command),
            env=None if env is None else dict(env),
            cwd=cwd,
            stdout=stdout,
            stderr=stderr,
            start_new_session=True,
        )
        self.pgid = self.process.pid
        self.sid = self.process.pid
        self.observed_pids: set[int] = {self.process.pid}
        self._record_members()

    def _record_members(self) -> list[ProcessRecord]:
        records = process_group_records(self.pgid)
        self.observed_pids.update(record.pid for record in records)
        return records

    def _reap_owned(self) -> None:
        for pid in sorted(self.observed_pids):
            if pid == self.process.pid:
                continue
            try:
                os.waitpid(pid, os.WNOHANG)
            except (ChildProcessError, ProcessLookupError):
                pass

    def _live_members(self) -> list[ProcessRecord]:
        records = self._record_members()
        return [record for record in records if record.state != "Z"]

    def wait_for_leader(self, timeout: float) -> int | None:
        try:
            return self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def _wait_group_empty(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._reap_owned()
            if not self._live_members():
                self._reap_owned()
                return True
            time.sleep(0.02)
        self._reap_owned()
        return not self._live_members()

    def _signal_group(self, sig: signal.Signals) -> None:
        self._record_members()
        try:
            os.killpg(self.pgid, sig)
        except ProcessLookupError:
            pass

    def shutdown(
        self,
        *,
        graceful_timeout: float = 3.0,
        term_timeout: float = 2.0,
        kill_timeout: float = 2.0,
    ) -> ShutdownResult:
        """Stop the owned group using SIGINT, SIGTERM, then SIGKILL if needed."""

        phase = "already_exited"
        if self._live_members():
            phase = "graceful"
            self._signal_group(signal.SIGINT)
            if not self._wait_group_empty(graceful_timeout):
                phase = "term"
                self._signal_group(signal.SIGTERM)
                if not self._wait_group_empty(term_timeout):
                    phase = "kill"
                    self._signal_group(signal.SIGKILL)
                    self._wait_group_empty(kill_timeout)

        self._reap_owned()
        try:
            leader_returncode = self.process.wait(timeout=0.1)
        except subprocess.TimeoutExpired:
            leader_returncode = self.process.poll()
        self._reap_owned()
        remaining = tuple(record.pid for record in self._live_members())
        return ShutdownResult(
            success=not remaining,
            phase=phase,
            pgid=self.pgid,
            observed_pids=tuple(sorted(self.observed_pids)),
            remaining_pids=remaining,
            leader_returncode=leader_returncode,
        )
