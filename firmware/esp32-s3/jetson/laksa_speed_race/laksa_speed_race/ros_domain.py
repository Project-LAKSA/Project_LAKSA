"""Validated, collision-aware ROS domain allocation for isolated C1 runs."""

from __future__ import annotations

from dataclasses import dataclass
import fcntl
from pathlib import Path
from typing import Callable, IO


VERIFIED_DOMAIN_MIN = 0
VERIFIED_DOMAIN_MAX = 232
# Keep C1 away from the default/production domain while retaining 33 candidates.
C1_DOMAIN_MIN = 200
C1_DOMAIN_MAX = VERIFIED_DOMAIN_MAX
C1_DOMAIN_COUNT = C1_DOMAIN_MAX - C1_DOMAIN_MIN + 1


class RosDomainError(ValueError):
    """A supplied or allocated ROS domain is invalid or unavailable."""


def parse_domain_id(value: object, *, source: str) -> int:
    """Parse and validate a domain before any ROS subprocess is started."""

    if isinstance(value, bool):
        raise RosDomainError(
            f"invalid ROS_DOMAIN_ID {value!r} from {source}; allowed integer range is "
            f"{VERIFIED_DOMAIN_MIN}..{VERIFIED_DOMAIN_MAX}"
        )
    try:
        text = str(value).strip()
        if not text:
            raise ValueError("empty")
        domain_id = int(text, 10)
    except (TypeError, ValueError) as error:
        raise RosDomainError(
            f"invalid ROS_DOMAIN_ID {value!r} from {source}; allowed integer range is "
            f"{VERIFIED_DOMAIN_MIN}..{VERIFIED_DOMAIN_MAX}"
        ) from error
    if not VERIFIED_DOMAIN_MIN <= domain_id <= VERIFIED_DOMAIN_MAX:
        raise RosDomainError(
            f"invalid ROS_DOMAIN_ID {domain_id} from {source}; allowed integer range is "
            f"{VERIFIED_DOMAIN_MIN}..{VERIFIED_DOMAIN_MAX}"
        )
    return domain_id


def bounded_c1_domain(seed: int) -> int:
    """Map every integer seed into the high, non-production C1 domain pool."""

    return C1_DOMAIN_MIN + (int(seed) % C1_DOMAIN_COUNT)


@dataclass
class RosDomainLease:
    domain_id: int
    source: str
    lock_path: Path
    _stream: IO[str]

    def release(self) -> None:
        if self._stream.closed:
            return
        fcntl.flock(self._stream.fileno(), fcntl.LOCK_UN)
        self._stream.close()

    def __enter__(self) -> "RosDomainLease":
        return self

    def __exit__(self, *_args: object) -> None:
        self.release()


def _try_lock(domain_id: int, lock_root: Path) -> RosDomainLease | None:
    lock_root.mkdir(parents=True, exist_ok=True)
    lock_path = lock_root / f"domain-{domain_id}.lock"
    stream = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        return None
    return RosDomainLease(domain_id, "", lock_path, stream)


def claim_explicit_domain(
    value: object,
    *,
    source: str,
    occupied: Callable[[int], bool],
    lock_root: Path,
) -> RosDomainLease:
    domain_id = parse_domain_id(value, source=source)
    lease = _try_lock(domain_id, lock_root)
    if lease is None:
        raise RosDomainError(f"ROS domain {domain_id} from {source} is already leased")
    lease.source = source
    try:
        if occupied(domain_id):
            raise RosDomainError(f"ROS domain {domain_id} from {source} has an active ROS graph")
        return lease
    except Exception:
        lease.release()
        raise


def allocate_isolated_domain(
    seed: int,
    *,
    occupied: Callable[[int], bool],
    lock_root: Path,
) -> RosDomainLease:
    """Lease the first empty C1 domain in a deterministic bounded rotation."""

    start = bounded_c1_domain(seed)
    for offset in range(C1_DOMAIN_COUNT):
        domain_id = C1_DOMAIN_MIN + ((start - C1_DOMAIN_MIN + offset) % C1_DOMAIN_COUNT)
        lease = _try_lock(domain_id, lock_root)
        if lease is None:
            continue
        lease.source = f"allocator(seed={seed})"
        try:
            if occupied(domain_id):
                lease.release()
                continue
            return lease
        except Exception:
            lease.release()
            raise
    raise RosDomainError(
        f"no empty ROS domain is available in isolated C1 pool {C1_DOMAIN_MIN}..{C1_DOMAIN_MAX}"
    )
