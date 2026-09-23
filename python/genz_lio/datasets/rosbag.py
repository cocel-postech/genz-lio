"""Reading rosbag1 and rosbag2 sequences.

Uses `rosbags`, which reads both without a ROS installation, so a sequence can
be replayed on a machine that has none.
"""
from __future__ import annotations

from pathlib import Path
from typing import Iterator, Optional
import warnings

import numpy as np

from . import Dataset, Frame
from .pointcloud2 import read_pointcloud2
from .livox import read_livox, make_livox_decoder
from .image import IMAGE_TYPES, CameraFrames
from ..genz_lio_pybind import PreprocessConfig, _ScanBuffer

_POINTCLOUD_TYPES = ("sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2")
_LIVOX_TYPES = ("livox_ros_driver/msg/CustomMsg", "livox_ros_driver/CustomMsg",
                "livox_ros_driver2/msg/CustomMsg", "livox_ros_driver2/CustomMsg")
_IMU_TYPES = ("sensor_msgs/msg/Imu", "sensor_msgs/Imu")


def _stamp(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


class RosbagDataset(Dataset):
    """One scan at a time, each with the IMU samples spanning it.

    A scan is only handed over once every IMU sample up to its end has been
    read, mirroring how the ROS wrappers pair the two live streams.
    """

    def __init__(self, path: str | Path, lidar_topic: Optional[str] = None,
                 imu_topic: Optional[str] = None, time_offset: float = 0.0,
                 preprocess: Optional[PreprocessConfig] = None,
                 image_topic: Optional[str] = None) -> None:
        try:
            from rosbags.highlevel import AnyReader
        except ImportError as error:  # pragma: no cover - depends on the extra
            raise ImportError(
                "reading rosbags needs the 'rosbag' extra: pip install 'genz-lio[rosbag]'"
            ) from error

        self._reader_factory = AnyReader
        self.path = Path(path)
        self.sequence_id = self.path.stem
        self.time_offset = time_offset
        self.preprocess = preprocess or PreprocessConfig()
        self.image_topic = image_topic
        if image_topic:
            try:
                import PIL.Image
            except ImportError as error:
                raise ImportError("camera preview requires Pillow: pip install 'genz-lio[viz]'") from error

        with AnyReader([self.path]) as reader:
            self.lidar_topic = lidar_topic or self._pick(reader, _POINTCLOUD_TYPES + _LIVOX_TYPES, "point cloud")
            self.imu_topic = imu_topic or self._pick(reader, _IMU_TYPES, "IMU")
            self._length = sum(c.msgcount for c in reader.connections
                               if c.topic == self.lidar_topic)
            if image_topic and not any(c.topic == image_topic and c.msgtype in IMAGE_TYPES
                                       for c in reader.connections):
                choices = sorted({c.topic for c in reader.connections if c.msgtype in IMAGE_TYPES})
                raise ValueError(f"no Image/CompressedImage topic {image_topic!r}; available: {choices}")

    @staticmethod
    def _pick(reader, types, what: str) -> str:
        topics = sorted({c.topic for c in reader.connections if c.msgtype in types})
        if not topics:
            raise ValueError(f"no {what} topic in the bag")
        if len(topics) > 1:
            raise ValueError(
                f"several {what} topics ({', '.join(topics)}); choose one explicitly"
            )
        return topics[0]

    def __len__(self) -> int:
        return self._length

    def __iter__(self) -> Iterator[Frame]:
        self.empty_clouds = 0
        self.pending_scans = 0
        image_topic = getattr(self, 'image_topic', None)
        camera = CameraFrames() if image_topic else None
        with self._reader_factory([self.path]) as reader:
            connections = [c for c in reader.connections
                           if c.topic in (self.lidar_topic, self.imu_topic, image_topic)]

            buffer = _ScanBuffer(self.preprocess)
            livox_decoders = {c.msgtype: make_livox_decoder(reader, c.msgtype)
                              for c in connections if c.msgtype in _LIVOX_TYPES}
            for connection, recorded_stamp, raw in reader.messages(connections=connections):
                decoder = livox_decoders.get(connection.msgtype)
                decoded = decoder(raw) if decoder is not None else None
                message = None if decoded is not None else reader.deserialize(raw, connection.msgtype)
                if camera is not None and connection.topic == image_topic:
                    stamp = _stamp(message.header)
                    camera.push(stamp if stamp else recorded_stamp * 1e-9, message)
                    continue
                if connection.topic == self.imu_topic:
                    event = buffer.push_imu(np.array([[
                        _stamp(message.header),
                        message.linear_acceleration.x, message.linear_acceleration.y,
                        message.linear_acceleration.z,
                        message.angular_velocity.x, message.angular_velocity.y,
                        message.angular_velocity.z,
                    ]]))
                else:
                    if decoded is not None:
                        stamp, points, timestamps, intensities, rings = decoded
                    elif connection.msgtype in _LIVOX_TYPES:
                        points, timestamps, intensities, rings = read_livox(message)
                    else:
                        points, timestamps, intensities, rings = read_pointcloud2(
                            message, self.preprocess.lidar_type)
                    if len(points) == 0:
                        self.empty_clouds += 1
                        continue
                    stamp = stamp if decoded is not None else _stamp(message.header)
                    event = buffer.push_scan(points, stamp + self.time_offset,
                                             timestamps, intensities, rings)
                if event in (1, 2):
                    # A single output trajectory cannot represent multiple time epochs.
                    raise ValueError("sensor timestamps went backwards; split the bag into monotonic segments")
                if event == 3:
                    warnings.warn("scan has no usable point timing; scan skipped", RuntimeWarning)
                while True:
                    bundle = buffer.pop()
                    if bundle is None:
                        break
                    points, begin, end, imu, times, intensities, rings, _ = bundle
                    camera_image, camera_timestamp = camera.sample(end) if camera else (None, None)
                    yield Frame(points, begin, end, imu, times, intensities, rings,
                                timing_prepared=True, camera_image=camera_image,
                                camera_timestamp=camera_timestamp)
            self.pending_scans = int(buffer.pending_scans)
            if self.pending_scans:
                warnings.warn(f"{self.pending_scans} scan(s) lack IMU coverage at end of bag; skipped",
                              RuntimeWarning)
