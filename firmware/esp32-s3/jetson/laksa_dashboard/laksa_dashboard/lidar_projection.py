"""Project a real planar LaserScan through its stamped rigid transform."""

from __future__ import annotations

import math


def _rotate(vector, quaternion):
    x, y, z = vector
    qx, qy, qz, qw = quaternion
    # Quaternion-vector rotation without Euler-angle assumptions.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def project_scan(ranges, angle_min, angle_increment, range_min, range_max, translation, quaternion):
    """Return transformed XYZ points; no vertical extrusion or invented geometry."""
    points = []
    tx, ty, tz = translation
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        rotated = _rotate((distance * math.cos(angle), distance * math.sin(angle), 0.0), quaternion)
        points.extend((rotated[0] + tx, rotated[1] + ty, rotated[2] + tz))
    return points
