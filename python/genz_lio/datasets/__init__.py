"""Readers for the formats LiDAR-inertial sequences come in.

A reader yields `Frame`s: one scan with the IMU samples spanning it. That is the
unit the pipeline consumes, and keeping the pairing inside the reader means the
formats' differing conventions — where the message stamp sits relative to the
sweep, what the per-point time is relative to — are resolved in one place.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

import numpy as np


@dataclass
class Frame:
    """One scan and the inertial samples that span it."""

    points: np.ndarray                  # (N, 3), LiDAR frame
    begin_time: float                   # seconds
    end_time: float                     # seconds
    imu: np.ndarray                     # (M, 7): timestamp, ax, ay, az, gx, gy, gz
    timestamps: Optional[np.ndarray] = None   # (N,) per-point times, as reported
    intensities: Optional[np.ndarray] = None  # (N,)
    rings: Optional[np.ndarray] = None        # (N,)
    timing_prepared: bool = False             # validated before IMU selection
    camera_image: Optional[np.ndarray] = None  # display-only RGB8 preview
    camera_timestamp: Optional[float] = None


class Dataset:
    """Base class. Iterating yields `Frame`s in order."""

    sequence_id: str = "sequence"

    def __iter__(self) -> Iterator[Frame]:
        raise NotImplementedError

    def __len__(self) -> int:
        raise NotImplementedError


def open_dataset(path: str | Path, **kwargs) -> Dataset:
    """Pick a reader from what `path` looks like.

    A rosbag1 file, a rosbag2 directory, an Ouster pcap, or a directory of scans
    with an IMU file beside them.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)

    if path.is_file() and path.suffix == ".bag":
        from .rosbag import RosbagDataset

        return RosbagDataset(path, **kwargs)
    if path.is_dir() and (path / "metadata.yaml").exists():
        from .rosbag import RosbagDataset

        return RosbagDataset(path, **kwargs)
    if path.is_file() and path.suffix == ".pcap":
        from .ouster import OusterDataset

        return OusterDataset(path, **kwargs)
    if path.is_dir():
        from .folder import FolderDataset

        return FolderDataset(path, **kwargs)

    raise ValueError(
        f"cannot tell what {path} is. Expected a .bag file, a rosbag2 directory, "
        "an Ouster .pcap, or a directory of scans with an IMU file."
    )


__all__ = ["Dataset", "Frame", "open_dataset"]
