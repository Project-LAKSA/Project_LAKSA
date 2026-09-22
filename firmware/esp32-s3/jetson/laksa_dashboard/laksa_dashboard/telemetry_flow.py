"""Small state machines for bounded Field Lab visualization data flow."""

from collections import deque


EPHEMERAL_TYPES = ("pose", "cloud", "lidar_slice", "safe_goal_mask", "occupancy", "ab_occupancy_zed", "ab_occupancy_hybrid")
EPHEMERAL_PRIORITY = ("pose", "lidar_slice", "safe_goal_mask", "occupancy", "ab_occupancy_zed", "ab_occupancy_hybrid", "cloud")


class OutgoingCoalescer:
    """Bounded reliable queue plus one latest value per ephemeral type."""

    def __init__(self, reliable_limit: int = 64):
        self._reliable = deque(maxlen=reliable_limit)
        self._latest = {}
        self.coalesced = {name: 0 for name in EPHEMERAL_TYPES}
        self.reliable_overflow = 0

    def put(self, payload: dict) -> None:
        message_type = payload.get("type")
        if message_type in EPHEMERAL_TYPES:
            if message_type in self._latest:
                self.coalesced[message_type] += 1
            self._latest[message_type] = payload
            return
        if len(self._reliable) == self._reliable.maxlen:
            self.reliable_overflow += 1
        self._reliable.append(payload)

    def pop(self):
        if self._reliable:
            return self._reliable.popleft()
        for message_type in EPHEMERAL_PRIORITY:
            if message_type in self._latest:
                return self._latest.pop(message_type)
        return None

    def clear_ephemeral(self) -> None:
        self._latest.clear()

    def __bool__(self) -> bool:
        return bool(self._reliable or self._latest)


class LatestRevisionGate:
    """Permit one active job and retain only the newest pending revision."""

    def __init__(self):
        self.active = None
        self.pending = None
        self.coalesced = 0

    def request(self, revision: int, value):
        item = (revision, value)
        if self.active is None:
            self.active = item
            return item
        if self.pending is not None:
            self.coalesced += 1
        self.pending = item
        return None

    def complete(self, revision: int):
        if self.active is None or self.active[0] != revision:
            raise RuntimeError("safe-mask completion does not match active revision")
        publish_completed = self.pending is None
        self.active = self.pending
        self.pending = None
        return publish_completed, self.active


def uniform_sample_indices(source_count: int, target_count: int) -> list[int]:
    """Return exactly target_count deterministic indices spanning the source."""
    if source_count <= 0 or target_count <= 0:
        return []
    if source_count <= target_count:
        return list(range(source_count))
    if target_count == 1:
        return [0]
    return [index * (source_count - 1) // (target_count - 1) for index in range(target_count)]
