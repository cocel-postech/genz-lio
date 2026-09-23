"""Reading a PointCloud2 without assuming what the driver called its fields.

Every driver in the benchmark differs only in what it names its timing field and
whether that timing counts forward from the start of the sweep or back from its
end. Both are detected here, so the reader hands the pipeline the same thing
either way — which is exactly what the C++ wrapper does with live messages.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

# Names a driver might give the per-point timing, in the order we prefer them.
_TIME_FIELDS = ("time", "t", "timestamp", "time_stamp", "offset_time")
_RING_FIELDS = ("ring", "line", "channel")

# Any stamp beyond this is a wall-clock time rather than an offset into the scan.
_ABSOLUTE_TIME_THRESHOLD = 1e6

_DATATYPES = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def _structured(msg) -> np.ndarray:
    """View the raw message buffer as a structured array, padding included."""
    if msg.width == 0 or msg.height == 0:
        return np.empty(0, dtype=[("x", float), ("y", float), ("z", float)])
    row_bytes = msg.width * msg.point_step
    required = (msg.height - 1) * msg.row_step + row_bytes
    if msg.point_step <= 0 or msg.row_step < row_bytes or required > len(msg.data):
        raise ValueError("invalid PointCloud2 stride or truncated data")
    names, formats, offsets = [], [], []
    endian = ">" if msg.is_bigendian else "<"
    for field in msg.fields:
        if field.datatype not in _DATATYPES or field.count < 1:
            raise ValueError(f"invalid PointCloud2 field: {field.name}")
        dtype = np.dtype(_DATATYPES[field.datatype]).newbyteorder(endian)
        if field.offset < 0 or field.offset + dtype.itemsize * field.count > msg.point_step:
            raise ValueError(f"out of bounds PointCloud2 field: {field.name}")
        names.append(field.name)
        formats.append(dtype if field.count == 1 else (dtype, (field.count,)))
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=msg.point_step))
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.row_step, msg.point_step)).reshape(-1)


def _first_present(names, available) -> Optional[str]:
    return next((name for name in names if name in available), None)


def read_pointcloud2(msg, lidar_type=None) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray],
                                   Optional[np.ndarray]]:
    """Return (points, timestamps, intensities, rings).

    `timestamps` are seconds relative to the message header, including any
    first-point/header offset, and None when usable timing is absent.
    """
    array = _structured(msg)
    names = array.dtype.names

    if not all(name in names for name in ("x", "y", "z")):
        raise ValueError("point cloud has no x/y/z fields")
    points = np.stack([array["x"], array["y"], array["z"]], axis=-1).astype(np.float64)
    finite = np.isfinite(points).all(axis=1)

    intensity_name = _first_present(("intensity", "reflectivity"), names)
    intensities = array[intensity_name].astype(np.float32) if intensity_name else None

    ring_name = _first_present(_RING_FIELDS, names)
    rings = array[ring_name].astype(np.int32) if ring_name else None

    time_name = _first_present(_TIME_FIELDS, names)
    timestamps = None
    if time_name:
        raw = array[time_name]
        # Integer timing fields count nanoseconds; floating-point ones seconds.
        scale = 1e-9 if np.issubdtype(raw.dtype, np.integer) else 1.0
        timestamps = raw.astype(np.float64) * scale
        # Enum names keep this decoder usable independently of the extension.
        sensor = getattr(lidar_type, "name", str(lidar_type)).lower()
        stamp = msg.header.stamp
        header_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        absolute_field = time_name in ("timestamp", "time_stamp")
        if sensor == "livox_pcl" and absolute_field:
            timestamps = (raw.astype(np.float64) - header_time * 1e9) * 1e-9
        elif absolute_field and sensor in ("hesai", "robosense"):
            timestamps -= header_time
        else:
            absolute = np.abs(timestamps) > _ABSOLUTE_TIME_THRESHOLD
            timestamps[absolute] -= header_time
        finite &= np.isfinite(timestamps) & (np.abs(timestamps) <= np.finfo(np.float32).max)
        usable = timestamps[finite]
        if usable.size == 0 or usable.max() <= usable.min():
            timestamps = None

    points = points[finite]
    if timestamps is not None:
        timestamps = timestamps[finite]
    if intensities is not None:
        intensities = intensities[finite]
    if rings is not None:
        rings = rings[finite]
    return points, timestamps, intensities, rings
