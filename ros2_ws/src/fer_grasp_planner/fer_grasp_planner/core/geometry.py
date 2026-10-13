"""Poses, boxes and quaternion / rotation-matrix conversions shared by the planners."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

Vector = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w


@dataclass(frozen=True)
class Box:
    position: Vector
    orientation: Quaternion
    size: Vector


@dataclass(frozen=True)
class Pose:
    position: Vector
    orientation: Quaternion


def matrix_from_quaternion(q: Quaternion) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_from_matrix(m: np.ndarray) -> Quaternion:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s,
             s / 4.0)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (s / 4.0, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
             (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, s / 4.0, (m[1, 2] + m[2, 1]) / s,
             (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4.0,
             (m[1, 0] - m[0, 1]) / s)
    return tuple(float(v) for v in q)
