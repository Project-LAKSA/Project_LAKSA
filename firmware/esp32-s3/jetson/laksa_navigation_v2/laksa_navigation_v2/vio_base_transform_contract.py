"""Dependency-free rigid-transform proof used by G2.1 static tests."""
from __future__ import annotations
import math


def quaternion_multiply(a, b):
    return (a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1], a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0], a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3], a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2])


def rotate(quaternion, vector):
    x, y, z, _ = quaternion_multiply(quaternion_multiply(quaternion, (*vector, 0.0)), (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]))
    return x, y, z


def compose(odom_to_camera_translation, odom_to_camera_quaternion, camera_to_base_translation, camera_to_base_quaternion):
    if not all(math.isfinite(v) for v in (*odom_to_camera_translation, *odom_to_camera_quaternion, *camera_to_base_translation, *camera_to_base_quaternion)):
        raise ValueError("non-finite transform")
    offset = rotate(odom_to_camera_quaternion, camera_to_base_translation)
    return (tuple(a+b for a, b in zip(odom_to_camera_translation, offset)), quaternion_multiply(odom_to_camera_quaternion, camera_to_base_quaternion))
