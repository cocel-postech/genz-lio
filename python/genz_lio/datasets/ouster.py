"""Single-sensor Ouster pcap recordings, including native IMU packets."""
from __future__ import annotations

from pathlib import Path
from typing import Iterator, Optional

import numpy as np

from . import Dataset, Frame


def _imu_samples(source, client, metadata):
    packet_format = client.PacketFormat.from_info(metadata)
    last_time = -np.inf
    for _, packet in source:
        if not isinstance(packet, client.ImuPacket):
            continue
        buf = packet.buf
        # Measurement timestamps share the LiDAR sensor clock. Packet system
        # timestamps describe packet creation, not the measured sample.
        stamp = 0.5e-9 * (packet_format.imu_accel_ts(buf) + packet_format.imu_gyro_ts(buf))
        sample = np.array([
            stamp,
            *(9.80665 * np.array([packet_format.imu_la_x(buf), packet_format.imu_la_y(buf),
                                 packet_format.imu_la_z(buf)])),
            *np.deg2rad([packet_format.imu_av_x(buf), packet_format.imu_av_y(buf),
                        packet_format.imu_av_z(buf)]),
        ])
        if not np.isfinite(sample).all() or stamp <= 0:
            continue
        if stamp < last_time:
            raise ValueError("Ouster IMU timestamps rewind; split the recording into monotonic runs")
        if stamp == last_time:
            continue
        last_time = stamp
        yield sample


class OusterDataset(Dataset):
    def __init__(self, path: str | Path, metadata: Optional[str] = None) -> None:
        try:
            from ouster.sdk import client, open_source
            from ouster.sdk.pcap import PcapMultiPacketReader
            from ouster.sdk.util import resolve_metadata
        except ImportError as error:  # pragma: no cover - depends on the extra
            raise ImportError(
                "reading Ouster pcap needs the 'ouster' extra: pip install 'genz-lio[ouster]'"
            ) from error

        self._client = client
        self._open_source = open_source
        self._packet_reader = PcapMultiPacketReader
        self.path = Path(path)
        self.sequence_id = self.path.stem
        self._metadata_path = resolve_metadata(str(self.path), metadata)
        if self._metadata_path is None:
            raise ValueError(f"no Ouster metadata JSON beside {self.path}")
        self._metadata = client.SensorInfo(Path(self._metadata_path).read_text())
        self.xyz_lut = client.XYZLut(self._metadata)
        # Index once for CLI progress; do not retain an open capture between runs.
        source = self._scans(index=True)
        try:
            self._length = len(source)
        finally:
            source.close()

    def _scans(self, index=False):
        return self._open_source(str(self.path), meta=[self._metadata_path], index=index)

    def __len__(self) -> int:
        return self._length

    def __iter__(self) -> Iterator[Frame]:
        scans = self._scans()
        packets = self._packet_reader(str(self.path), [], metadatas=[self._metadata])
        try:
            samples = iter(_imu_samples(packets, self._client, self._metadata))
            following = next(samples, None)
            if following is None:
                raise ValueError(f"{self.path} carries no valid IMU packets; LiDAR alone is insufficient")
            previous_end = -np.inf
            for scan in scans:
                points = self.xyz_lut(scan).reshape(-1, 3).astype(np.float64)
                stamps = np.tile(scan.timestamp.astype(np.float64) * 1e-9, scan.h)
                columns_valid = np.tile((scan.status & 1) != 0, scan.h)
                valid = columns_valid & (stamps > 0) & np.isfinite(points).all(axis=1)
                valid &= np.linalg.norm(points, axis=1) > 0
                points, stamps = points[valid], stamps[valid]
                if not len(points):
                    continue
                begin, end = float(stamps.min()), float(stamps.max())
                if begin < previous_end:
                    raise ValueError("Ouster scan timestamps overlap or rewind; split the recording")
                previous_end = end
                frame_samples = []
                # Retain the next sample for the following scan. Include samples
                # between scans, which the inertial propagation also requires.
                while following is not None and following[0] <= end:
                    frame_samples.append(following)
                    following = next(samples, None)
                if following is None and (not frame_samples or frame_samples[-1][0] < end):
                    break  # no IMU coverage through the final scan
                yield Frame(points=points, begin_time=begin, end_time=end,
                            imu=np.asarray(frame_samples, dtype=np.float64).reshape(-1, 7),
                            timestamps=stamps - begin)
        finally:
            scans.close()
            packets.close()
