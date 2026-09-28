import os
import subprocess
import sys
import time

import pytest

from laksa_speed_race.owned_process_session import OwnedProcessSession, process_group_records


pytestmark = pytest.mark.skipif(
    not sys.platform.startswith("linux"), reason="process-group verification uses Linux /proc"
)


def _fixture(mode: str) -> list[str]:
    source = r"""
import os
import signal
import subprocess
import sys
import time

mode = sys.argv[1]
if mode in {"ignore_int", "ignore_term"}:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
if mode == "ignore_term":
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
if mode == "exception":
    raise RuntimeError("fixture exception")
if mode == "descendant":
    subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
while True:
    time.sleep(0.05)
"""
    return [sys.executable, "-c", source, mode]


def test_normal_successful_cleanup_has_no_orphans():
    session = OwnedProcessSession(_fixture("normal"))
    result = session.shutdown(graceful_timeout=1.0)
    assert result.success
    assert result.phase == "graceful"
    assert not process_group_records(result.pgid)


def test_controller_exception_cleanup():
    session = OwnedProcessSession(_fixture("exception"), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    assert session.wait_for_leader(timeout=2.0) != 0
    result = session.shutdown()
    assert result.success
    assert not result.remaining_pids


def test_timeout_cleanup():
    session = OwnedProcessSession(_fixture("normal"))
    assert session.wait_for_leader(timeout=0.05) is None
    assert session.shutdown(graceful_timeout=1.0).success


def test_term_fallback():
    session = OwnedProcessSession(_fixture("ignore_int"))
    time.sleep(0.1)
    result = session.shutdown(graceful_timeout=0.1, term_timeout=1.0)
    assert result.success
    assert result.phase == "term"


def test_kill_fallback():
    session = OwnedProcessSession(_fixture("ignore_term"))
    time.sleep(0.1)
    result = session.shutdown(graceful_timeout=0.1, term_timeout=0.1, kill_timeout=1.0)
    assert result.success
    assert result.phase == "kill"


def test_descendants_are_reaped():
    session = OwnedProcessSession(_fixture("descendant"))
    time.sleep(0.15)
    before = process_group_records(session.pgid)
    assert len(before) >= 2
    assert {record.pgid for record in before} == {session.pgid}
    assert {record.sid for record in before} == {session.sid}
    result = session.shutdown(graceful_timeout=1.0)
    assert result.success
    assert len(result.observed_pids) >= 2
    assert not process_group_records(result.pgid)


def test_unrelated_process_survives():
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        unrelated_pgid = os.getpgid(unrelated.pid)
        session = OwnedProcessSession(_fixture("descendant"))
        assert session.pgid != unrelated_pgid
        assert session.shutdown(graceful_timeout=1.0).success
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=2.0)


def test_repeated_sessions_have_unique_groups_and_no_stale_processes():
    first = OwnedProcessSession(_fixture("descendant"))
    first_pgid = first.pgid
    assert first.shutdown(graceful_timeout=1.0).success
    second = OwnedProcessSession(_fixture("descendant"))
    second_pgid = second.pgid
    assert second_pgid != first_pgid
    assert second.shutdown(graceful_timeout=1.0).success
    assert not process_group_records(first_pgid)
    assert not process_group_records(second_pgid)
