"""Writing a trajectory in the formats the evaluation tools expect."""
from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np


def _quaternion(rotation: np.ndarray) -> np.ndarray:
    """Rotation matrix to (x, y, z, w), via the largest-diagonal branch so that
    no denominator approaches zero."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([(m[2, 1] - m[1, 2]) * s, (m[0, 2] - m[2, 0]) * s,
                         (m[1, 0] - m[0, 1]) * s, 0.25 / s])
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                         (m[2, 1] - m[1, 2]) / s])
    if m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                         (m[0, 2] - m[2, 0]) / s])
    s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
    return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
                     (m[1, 0] - m[0, 1]) / s])


def write_tum(path: str | Path, timestamps: Sequence[float], poses: Sequence[np.ndarray]) -> None:
    """TUM format: `timestamp tx ty tz qx qy qz qw`, one pose per line."""
    with open(path, "w") as handle:
        for stamp, pose in zip(timestamps, poses):
            t = pose[:3, 3]
            q = _quaternion(pose[:3, :3])
            handle.write(f"{stamp:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} "
                         f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


def write_kitti(path: str | Path, poses: Sequence[np.ndarray]) -> None:
    """KITTI format: the first three rows of each transform, flattened."""
    with open(path, "w") as handle:
        for pose in poses:
            handle.write(" ".join(f"{value:.9e}" for value in pose[:3, :].ravel()) + "\n")
