"""Deterministic, bounded point-cloud sampling for Field Lab previews."""

from __future__ import annotations

import numpy as np


def spatial_voxel_sample_indices(xyz: np.ndarray, target_count: int) -> np.ndarray:
    """Return deterministic indices with one representative per spatial voxel.

    The voxel resolution is selected by binary search so the candidate set is
    just large enough for the requested budget.  This makes coverage depend on
    geometry rather than the producer's point ordering.
    """
    points = np.asarray(xyz)
    source_count = len(points)
    if source_count <= 0 or target_count <= 0:
        return np.empty(0, dtype=np.intp)
    if source_count <= target_count:
        return np.arange(source_count, dtype=np.intp)
    if target_count == 1:
        return np.asarray((0,), dtype=np.intp)

    minimum = np.min(points, axis=0)
    span = np.ptp(points, axis=0)
    span[span <= np.finfo(np.float64).eps] = 1.0
    normalized = (points - minimum) / span

    def representatives(divisions: int) -> np.ndarray:
        cells = np.minimum(
            (normalized * divisions).astype(np.int64), divisions - 1
        )
        keys = (
            cells[:, 0] * divisions * divisions
            + cells[:, 1] * divisions
            + cells[:, 2]
        )
        _, first = np.unique(keys, return_index=True)
        return first.astype(np.intp, copy=False)

    low = 1
    high = max(2, int(np.ceil(target_count ** (1.0 / 3.0))))
    candidate = representatives(high)
    while len(candidate) < target_count and high < 65536:
        low = high + 1
        high *= 2
        candidate = representatives(high)
    while low < high:
        middle = (low + high) // 2
        middle_candidate = representatives(middle)
        if len(middle_candidate) >= target_count:
            high = middle
            candidate = middle_candidate
        else:
            low = middle + 1
    if len(candidate) < target_count:
        # Exact duplicate coordinates can keep the voxel set below the budget.
        # Fill deterministically without duplicating point indices.
        remaining = np.setdiff1d(
            np.arange(source_count, dtype=np.intp), candidate, assume_unique=False
        )
        candidate = np.concatenate((candidate, remaining[: target_count - len(candidate)]))
    if len(candidate) > target_count:
        positions = np.linspace(0, len(candidate) - 1, target_count, dtype=np.intp)
        candidate = candidate[positions]
    return np.sort(candidate.astype(np.intp, copy=False))
