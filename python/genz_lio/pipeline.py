"""Driving a dataset through the odometry and collecting what comes out."""
from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np

from .datasets import Dataset
from .genz_lio_pybind import Config, GenZLIO, _VisualizationMap
from .tools.pose_writers import write_kitti, write_tum


@dataclass
class RunSummary:
    """What a run produced, beyond the trajectory itself."""

    sequence_id: str
    frames: int = 0
    skipped: int = 0
    stopped_early: bool = False
    path_length: float = 0.0
    wall_time: float = 0.0
    frame_times_ms: List[float] = field(default_factory=list)

    @property
    def mean_frame_time_ms(self) -> float:
        return float(np.mean(self.frame_times_ms)) if self.frame_times_ms else 0.0

    @property
    def p95_frame_time_ms(self) -> float:
        return float(np.percentile(self.frame_times_ms, 95)) if self.frame_times_ms else 0.0

    @property
    def real_time_factor(self) -> float:
        """Above 1 means the run kept up with the sensor."""
        if not self.frame_times_ms or self.wall_time <= 0:
            return 0.0
        return self.frames * 0.1 / self.wall_time


class Pipeline:
    """Runs one dataset start to finish.

    Frames the odometry cannot use — before the IMU has initialized, or with too
    few points — are counted as skipped rather than silently dropped, since a
    large count is usually a sign the configuration does not match the data.
    """

    def __init__(self, dataset: Dataset, config: Optional[Config] = None,
                 visualize: bool = False, visualize_autoplay: bool = False,
                 visualize_map_spacing: float = 0.) -> None:
        self.dataset = dataset
        self.config = config or Config()
        self.odometry = GenZLIO(self.config)
        self.timestamps: List[float] = []
        self.poses: List[np.ndarray] = []
        self.summary = RunSummary(sequence_id=getattr(dataset, "sequence_id", "sequence"))

        self._visualizer = None
        self._map_view = None
        self._processing = None
        if visualize:
            from .tools.visualizer import Visualizer

            self._visualizer = Visualizer(autoplay=visualize_autoplay, map_spacing=visualize_map_spacing,
                                          show_camera=bool(getattr(dataset, "image_topic", None)))
            self._map_view = _VisualizationMap(self.odometry)

    def run(self, progress=None) -> RunSummary:
        try:
            return self._run(progress)
        finally:
            if self._processing is not None:
                self._processing.close()
            if self._visualizer is not None:
                self._visualizer.close()

    def _processed_frames(self):
        """Prefetch at most one owned display packet while the GUI draws.

        The worker alone accesses the reader and estimator. OpenGL stays on the
        main thread; no estimator state is read there during a pending scan.
        Pausing stops further submissions, after at most one in-flight frame.
        """
        frames = None

        def process_next():
            nonlocal frames
            if frames is None:
                frames = iter(self.dataset)
            frame = next(frames, None)
            if frame is None:
                return None
            if len(frame.imu) == 0:
                return (frame, None, None)
            result = self.odometry.register_scan(
                frame.points, frame.begin_time, frame.end_time, frame.imu,
                timestamps=frame.timestamps, intensities=frame.intensities,
                rings=frame.rings, timing_prepared=frame.timing_prepared,
            )
            display = None
            if result.valid and self._visualizer is not None:
                display = (
                    self.odometry.lidar_pose.copy(), self.odometry.gravity_alignment.copy(),
                    self._map_view.update(getattr(self._visualizer, "map_spacing", 0.)) if self._map_view is not None
                    else self.odometry.visualization_snapshot,
                )
            return frame, result, display

        if self._map_view is None:
            while True:
                packet = process_next()
                if packet is None:
                    return
                yield packet
        else:
            with ThreadPoolExecutor(max_workers=1, thread_name_prefix="genz-lio") as worker:
                pending = None
                try:
                    while True:
                        if pending is None:
                            pending = worker.submit(process_next)
                        packet = pending.result()
                        pending = None
                        if packet is None:
                            return
                        if getattr(self._visualizer, "_play_mode", False):
                            pending = worker.submit(process_next)
                        yield packet
                finally:
                    if pending is not None:
                        pending.cancel()
                    # Bag readers may hold thread-affine database handles.
                    # Close their iterator on its owner, after any in-flight scan.
                    def close_reader():
                        close = getattr(frames, "close", None)
                        if close is not None:
                            close()
                    worker.submit(close_reader).result()

    def _run(self, progress=None) -> RunSummary:
        started = time.perf_counter()
        previous: Optional[np.ndarray] = None

        self._processing = self._processed_frames()
        for frame, result, display in self._processing:
            if progress is not None:
                progress()
            if result is None or not result.valid:
                self.summary.skipped += 1
                continue

            pose = result.pose
            self.timestamps.append(result.timestamp)
            self.poses.append(pose)
            self.summary.frames += 1
            self.summary.frame_times_ms.append(result.processing_time_ms)
            if previous is not None:
                self.summary.path_length += float(np.linalg.norm(pose[:3, 3] - previous[:3, 3]))
            previous = pose

            if self._visualizer is not None:
                lidar_pose, alignment, snapshot = display
                world_rotation = alignment @ lidar_pose[:3, :3]
                world_translation = alignment @ lidar_pose[:3, 3]
                world_points = result.deskewed @ world_rotation.T + world_translation
                display_pose = pose.copy()
                display_pose[:3, :3] = alignment @ pose[:3, :3]
                display_pose[:3, 3] = alignment @ pose[:3, 3]
                map_transform = np.eye(4)
                map_transform[:3, :3] = alignment
                infos = {name: getattr(result, name) for name in
                         ("plane_matches", "point_matches", "leaf_size", "setpoint",
                          "voxelized_points", "processing_time_ms", "scale_indicator")}
                # Compare the actual voxelizer input against its output. Organized
                # sensor frames may contain a constant number of empty returns.
                infos["raw_points"] = len(world_points)
                infos["input_points"] = len(frame.points)
                infos["scale_threshold"] = self.config.adaptive_voxelization.scale_threshold
                infos["distance_traveled"] = self.summary.path_length
                map_options = ({"map_updates": snapshot} if self._map_view is not None
                               else {"map_points": snapshot["map"]})
                if getattr(self.dataset, 'image_topic', None):
                    self._visualizer.set_camera_image(frame.camera_image, frame.camera_timestamp)
                keep_running = self._visualizer.update(
                    world_points, display_pose,
                    planar_points=snapshot["planar"] @ alignment.T,
                    non_planar_points=snapshot["non_planar"] @ alignment.T,
                    map_transform=map_transform, infos=infos, **map_options)
                if keep_running is False:
                    self.summary.stopped_early = True
                    break

        self.summary.wall_time = time.perf_counter() - started
        if self._visualizer is not None:
            self._visualizer.finish()
        return self.summary

    def save(self, directory: str | Path, formats=("tum",)) -> List[Path]:
        """Write the trajectory out; returns the files written."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        written = []
        for name in formats:
            if name == "tum":
                path = directory / f"{self.summary.sequence_id}_tum.txt"
                write_tum(path, self.timestamps, self.poses)
            elif name == "kitti":
                path = directory / f"{self.summary.sequence_id}_kitti.txt"
                write_kitti(path, self.poses)
            else:
                raise ValueError(f"unknown trajectory format {name!r}; expected tum or kitti")
            written.append(path)
        return written
