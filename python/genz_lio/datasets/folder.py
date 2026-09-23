"""A directory of scans with an IMU file beside them.

For sequences that are not distributed as bags. Scans are `.bin` (KITTI's
float32 x, y, z, intensity), `.npy`, or `.txt`; the IMU is a CSV of
`timestamp, ax, ay, az, gx, gy, gz`. Scan times come from `timestamps.txt`;
when it is absent, they start at the first IMU timestamp and advance by
`scan_period`. File names determine scan ordering, not timestamps.
"""
from __future__ import annotations

from pathlib import Path
from typing import Iterator, List, Optional

import numpy as np

from . import Dataset, Frame

_SCAN_SUFFIXES = (".bin", ".npy", ".txt")
_IMU_NAMES = ("imu.csv", "imu.txt", "imu_data.csv")


def _load_scan(path: Path) -> np.ndarray:
    if path.suffix == ".npy":
        points = np.load(path)
    elif path.suffix == ".bin":
        points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    else:
        points = np.loadtxt(path)
    return np.asarray(points[:, :3], dtype=np.float64)


class FolderDataset(Dataset):
    def __init__(self, path: str | Path, scan_period: float = 0.1,
                 imu_file: Optional[str] = None) -> None:
        self.path = Path(path)
        self.sequence_id = self.path.name
        self.scan_period = scan_period

        self.scans: List[Path] = sorted(
            p for p in self.path.iterdir()
            if p.suffix in _SCAN_SUFFIXES and p.name not in _IMU_NAMES
            and p.stem != "timestamps"
        )
        if not self.scans:
            raise ValueError(f"no scans in {self.path} (looked for {', '.join(_SCAN_SUFFIXES)})")

        imu_path = Path(imu_file) if imu_file else next(
            (self.path / name for name in _IMU_NAMES if (self.path / name).exists()), None)
        if imu_path is None:
            raise ValueError(
                f"no IMU file in {self.path}. GenZ-LIO is inertial-aided and cannot run "
                f"without one; expected one of {', '.join(_IMU_NAMES)}."
            )
        self.imu = np.atleast_2d(np.loadtxt(imu_path, delimiter=None if imu_path.suffix == ".txt"
                                            else ","))
        if self.imu.shape[1] != 7:
            raise ValueError(f"{imu_path} must have 7 columns: timestamp, ax, ay, az, gx, gy, gz")

        stamps_file = self.path / "timestamps.txt"
        if stamps_file.exists():
            self.times = np.loadtxt(stamps_file, dtype=float)
            if len(self.times) != len(self.scans):
                raise ValueError("timestamps.txt has a different length than the scan list")
        else:
            # Fall back to the IMU's own start, stepping by the scan period.
            start = float(self.imu[0, 0])
            self.times = start + np.arange(len(self.scans)) * scan_period

    def __len__(self) -> int:
        return len(self.scans)

    def __iter__(self) -> Iterator[Frame]:
        cursor = 0
        for index, scan_path in enumerate(self.scans):
            begin = float(self.times[index])
            end = begin + self.scan_period
            stop = np.searchsorted(self.imu[:, 0], end, side="right")
            frame_imu = self.imu[cursor:stop]
            cursor = int(stop)
            yield Frame(points=_load_scan(scan_path), begin_time=begin, end_time=end,
                        imu=frame_imu)
