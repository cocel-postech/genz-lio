"""Reading and writing the YAML configuration.

The same files the ROS wrappers read, so a run can be reproduced from either
side without translating anything. Keys the file omits keep the base or compiled
core defaults, and unknown keys are reported rather than ignored.
"""
from __future__ import annotations

import warnings
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import yaml

from .genz_lio_pybind import Config, LidarType

_LIDAR_TYPES = {
    "livox": LidarType.LIVOX,
    "livox_pcl": LidarType.LIVOX_PCL,
    "velodyne": LidarType.VELODYNE,
    "ouster": LidarType.OUSTER,
    "hesai": LidarType.HESAI,
    "robosense": LidarType.ROBOSENSE,
}

# Sections the ROS wrappers own; seeing them here is not a mistake.
_WRAPPER_SECTIONS = {"common", "publish", "pcd_save"}

_SCALARS = {
    "preprocess": ["scan_line", "scan_rate", "blind_min", "blind_max", "point_filter_num"],
    "mapping": [
        "max_iteration", "down_sample_size", "voxel_size", "max_layer", "planar_threshold",
        "sigma_num", "max_points_size", "max_mature_points_size", "map_range",
        "extrinsic_est_en",
    ],
    "adaptive_voxelization": [
        "enable", "window_size", "scale_threshold", "setpoint_exponent", "min_points",
        "max_points", "error_sensitivity", "error_rate_sensitivity", "p_gain_min",
        "p_gain_max", "d_gain_min", "d_gain_max",
    ],
    "hybrid_metric": [
        "enable", "lambda_po", "sigma_num",
        "max_points_per_voxel", "reduction_ratio",
    ],
    "noise_model": [
        "ranging_cov", "angle_cov", "acc_cov", "gyr_cov", "b_acc_cov", "b_gyr_cov",
    ],
}
_THRESHOLD_KEYS = [
    "initial_threshold",
    "max_range_motion",
]


def _apply(section: Dict[str, Any], target: Any, keys: List[str], prefix: str,
           unknown: List[str]) -> None:
    handled = set(keys)
    for key, value in section.items():
        if key in handled:
            setattr(target, key, value)
        elif key not in _EXTRA_KEYS.get(prefix, ()):
            unknown.append(f"{prefix}{key}")


_EXTRA_KEYS = {
    "preprocess/": ("lidar_type",),
    "mapping/": ("layer_point_size", "extrinsic_t", "extrinsic_r"),
    "hybrid_metric/": ("adaptive_threshold",),
}


def load_config(path: str | Path, base: Config | None = None) -> Config:
    """Parse `path` on top of `base`, or on top of the defaults.

    Shipped sensor defaults are complete and need only one load:

        config = load_config("ros/config/default/ouster.yaml")

    An explicit `base` still supports custom/legacy partial overlays.
    """
    config = base if base is not None else Config()
    with open(path) as handle:
        root = yaml.safe_load(handle) or {}
    if not isinstance(root, dict):
        raise ValueError(f"{path} is not a mapping")

    unknown: List[str] = []

    if "runtime" in root:
        for key, value in (root["runtime"] or {}).items():
            if key == "max_threads":
                config.max_threads = value
            else:
                unknown.append(f"runtime/{key}")

    for name, keys in _SCALARS.items():
        section = root.get(name)
        if not section:
            continue
        _apply(section, getattr(config, name), keys, f"{name}/", unknown)

    preprocess = root.get("preprocess") or {}
    if "lidar_type" in preprocess:
        name = preprocess["lidar_type"]
        if name not in _LIDAR_TYPES:
            raise ValueError(f"unknown lidar_type {name!r}; expected one of "
                             f"{', '.join(sorted(_LIDAR_TYPES))}")
        config.preprocess.lidar_type = _LIDAR_TYPES[name]

    mapping = root.get("mapping") or {}
    if "layer_point_size" in mapping:
        config.mapping.layer_point_size = list(mapping["layer_point_size"])
    if "extrinsic_t" in mapping:
        values = mapping["extrinsic_t"]
        if len(values) != 3:
            raise ValueError("mapping/extrinsic_t expects 3 values")
        config.mapping.extrinsic_t = np.asarray(values, dtype=float)
    if "extrinsic_r" in mapping:
        values = mapping["extrinsic_r"]
        if len(values) != 9:
            raise ValueError("mapping/extrinsic_r expects 9 values, row-major")
        config.mapping.extrinsic_r = np.asarray(values, dtype=float).reshape(3, 3)

    hybrid = root.get("hybrid_metric") or {}
    if "adaptive_threshold" in hybrid:
        _apply(hybrid["adaptive_threshold"] or {}, config.hybrid_metric.adaptive_threshold,
               _THRESHOLD_KEYS, "hybrid_metric/adaptive_threshold/", unknown)

    known = set(_SCALARS) | {"runtime"} | _WRAPPER_SECTIONS
    unknown += [f"section {name}" for name in root if name not in known]

    if unknown:
        warnings.warn(f"{path}: unrecognised keys: {', '.join(unknown)}", stacklevel=2)
    return config


def save_config(config: Config, path: str | Path) -> None:
    """Write a configuration back out, in the layout the shipped files use."""
    lidar = next(k for k, v in _LIDAR_TYPES.items() if v == config.preprocess.lidar_type)
    threshold = config.hybrid_metric.adaptive_threshold

    document = {
        "runtime": {"max_threads": config.max_threads},
        "preprocess": {"lidar_type": lidar,
                       **{k: getattr(config.preprocess, k) for k in _SCALARS["preprocess"]}},
        "mapping": {
            **{k: getattr(config.mapping, k) for k in _SCALARS["mapping"]},
            "layer_point_size": list(config.mapping.layer_point_size),
            "extrinsic_t": np.asarray(config.mapping.extrinsic_t).ravel().tolist(),
            "extrinsic_r": np.asarray(config.mapping.extrinsic_r).ravel().tolist(),
        },
        "adaptive_voxelization": {k: getattr(config.adaptive_voxelization, k)
                                  for k in _SCALARS["adaptive_voxelization"]},
        "hybrid_metric": {
            **{k: getattr(config.hybrid_metric, k) for k in _SCALARS["hybrid_metric"]},
            "adaptive_threshold": {k: getattr(threshold, k) for k in _THRESHOLD_KEYS},
        },
        "noise_model": {k: getattr(config.noise_model, k) for k in _SCALARS["noise_model"]},
    }
    with open(path, "w") as handle:
        yaml.safe_dump(document, handle, sort_keys=False, default_flow_style=False)
