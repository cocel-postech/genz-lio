"""GenZ-LIO: generalizable LiDAR-inertial odometry.

The estimator itself is the C++ core; this package wraps it, adds readers for
the formats the benchmark sequences come in, and a command-line pipeline.

    import numpy as np
    from genz_lio import Config, GenZLIO

    lio = GenZLIO(Config())
    result = lio.register_scan(points, scan_begin, scan_end, imu, timestamps=times)
    print(result.pose)
"""
from .genz_lio_pybind import (  # noqa: F401
    AdaptiveThresholdConfig,
    AdaptiveVoxelizationConfig,
    Config,
    GenZLIO,
    HybridMetricConfig,
    LidarType,
    MappingConfig,
    NoiseModelConfig,
    PreprocessConfig,
    Result,
    Timing,
    __version__,
)
from .config import load_config, save_config  # noqa: F401

__all__ = [
    "AdaptiveThresholdConfig",
    "AdaptiveVoxelizationConfig",
    "Config",
    "GenZLIO",
    "HybridMetricConfig",
    "LidarType",
    "MappingConfig",
    "NoiseModelConfig",
    "PreprocessConfig",
    "Result",
    "Timing",
    "load_config",
    "save_config",
    "__version__",
]
