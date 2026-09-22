"""Pose-independent conservative safe-center masks for OccupancyGrid data."""

from __future__ import annotations

import base64
import math

try:
    import numpy as np
except ImportError:  # Exact pure-Python fallback remains fully supported.
    np = None

try:
    from scipy.ndimage import distance_transform_edt as scipy_distance_transform_edt
except ImportError:  # SciPy is optional and is never installed by this package.
    scipy_distance_transform_edt = None


def circumscribed_radius(front: float, rear: float, left: float, right: float) -> float:
    """Return the largest center-to-footprint-corner distance."""
    return max(
        math.hypot(front, left),
        math.hypot(front, right),
        math.hypot(rear, left),
        math.hypot(rear, right),
    )


def _edt_1d(values: list[float]) -> list[float]:
    """Exact squared Euclidean distance transform (Felzenszwalb/Huttenlocher)."""
    size = len(values)
    sites = [index for index, value in enumerate(values) if math.isfinite(value)]
    if not sites:
        return [math.inf] * size
    vertices = [0] * len(sites)
    boundaries = [0.0] * (len(sites) + 1)
    k = 0
    vertices[0] = sites[0]
    boundaries[0] = -math.inf
    boundaries[1] = math.inf
    for site in sites[1:]:
        while True:
            prior = vertices[k]
            crossing = ((values[site] + site * site) - (values[prior] + prior * prior)) / (2.0 * (site - prior))
            if crossing > boundaries[k] or k == 0:
                break
            k -= 1
        k += 1
        vertices[k] = site
        boundaries[k] = crossing
        boundaries[k + 1] = math.inf
    result = [0.0] * size
    k = 0
    for index in range(size):
        while boundaries[k + 1] < index:
            k += 1
        delta = index - vertices[k]
        result[index] = delta * delta + values[vertices[k]]
    return result


def build_safe_mask_reference(data: list[int], width: int, height: int, resolution: float, radius: float) -> tuple[bytes, int]:
    """Build an LSB-first bit mask; unknown and costs >=99 are blocked.

    A one-cell blocked border makes outside-map space participate in the
    distance transform. The cell half diagonal is added to the clearance
    threshold so a safe bit cannot rely on center-to-center clearance alone.
    """
    if width <= 0 or height <= 0 or len(data) != width * height or resolution <= 0.0:
        raise ValueError("invalid occupancy grid")
    padded_width = width + 2
    padded_height = height + 2
    blocked = [[0.0] * padded_width for _ in range(padded_height)]
    for row in range(1, padded_height - 1):
        source = (row - 1) * width
        for column in range(1, padded_width - 1):
            cost = data[source + column - 1]
            blocked[row][column] = 0.0 if cost < 0 or cost >= 99 else math.inf

    horizontal = [_edt_1d(row) for row in blocked]
    squared = [[0.0] * padded_width for _ in range(padded_height)]
    for column in range(padded_width):
        transformed = _edt_1d([horizontal[row][column] for row in range(padded_height)])
        for row, value in enumerate(transformed):
            squared[row][column] = value

    required_cells = radius / resolution + math.sqrt(0.5)
    threshold = required_cells * required_cells
    packed = bytearray((width * height + 7) // 8)
    safe_cells = 0
    for row in range(height):
        for column in range(width):
            source_index = row * width + column
            cost = data[source_index]
            if cost >= 0 and cost < 99 and squared[row + 1][column + 1] + 1.0e-12 >= threshold:
                packed[source_index >> 3] |= 1 << (source_index & 7)
                safe_cells += 1
    return bytes(packed), safe_cells


def _build_safe_mask_scipy(
    data: list[int], width: int, height: int, resolution: float, radius: float
) -> tuple[bytes, int]:
    """Native exact EDT with a minute conservative floating-point margin."""
    if width <= 0 or height <= 0 or len(data) != width * height or resolution <= 0.0:
        raise ValueError("invalid occupancy grid")
    costs = np.asarray(data, dtype=np.int16).reshape(height, width)
    free = (costs >= 0) & (costs < 99)
    padded = np.pad(free, 1, mode="constant", constant_values=False)
    distances = scipy_distance_transform_edt(padded)[1:-1, 1:-1]
    required_cells = radius / resolution + math.sqrt(0.5)
    # Requiring a tiny extra clearance guarantees that native rounding cannot
    # turn a reference-unsafe boundary cell into a safe one.
    safe = free & (distances >= required_cells + 1.0e-9)
    flat = safe.reshape(-1)
    packed = np.packbits(flat, bitorder="little")
    return packed.tobytes(), int(np.count_nonzero(flat))


def edt_backend() -> str:
    return "scipy.ndimage.distance_transform_edt" if scipy_distance_transform_edt is not None else "exact_python_felzenszwalb_huttenlocher"


def build_safe_mask(data: list[int], width: int, height: int, resolution: float, radius: float) -> tuple[bytes, int]:
    if scipy_distance_transform_edt is not None:
        return _build_safe_mask_scipy(data, width, height, resolution, radius)
    return build_safe_mask_reference(data, width, height, resolution, radius)


def encoded_mask(data: list[int], width: int, height: int, resolution: float, radius: float) -> tuple[str, int]:
    packed, safe_cells = build_safe_mask(data, width, height, resolution, radius)
    return base64.b64encode(packed).decode("ascii"), safe_cells


def mask_contains(packed: bytes, index: int) -> bool:
    return index >= 0 and (index >> 3) < len(packed) and bool(packed[index >> 3] & (1 << (index & 7)))
