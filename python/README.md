# GenZ-LIO Python

Run recorded LiDAR/IMU data through the C++ estimator, optionally with an
interactive Polyscope visualizer. ROS is not required for offline rosbag reading.

## Installation

The Python package targets Python 3.8–3.12 on Linux x86-64. Build dependencies
are a C++17 compiler, CMake 3.16+, OpenMP, Eigen 3.4+, and Boost headers.
The visualizer also needs an OpenGL-capable display.

Install from a complete checkout in a virtual environment:

```bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev libboost-dev \
    python3-dev python3-venv libgl1
git clone https://github.com/cocel-postech/genz-lio.git
cd genz-lio
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install '.[all]'
genz_lio_pipeline run --help
```

CMake downloads Eigen 3.4 if the system version is older; this needs network
access during the build. Packaging is configured by the repository-root
`pyproject.toml`. Run pip from that root so C++ sources, YAMLs, and licenses
are included. Installation is currently from source; no PyPI release is required.

Install `.` for the core and CLI only, `.[rosbag]` to read bags,
or `.[rosbag,viz]` for bag playback with visualization. The `all` extra includes rosbag, visualization, and Ouster SDK dependencies;
`ouster` installs the SDK reader dependencies. The SDK is pinned to its compatible
0.13 API; this extra requires glibc 2.28 or newer (for example, Ubuntu 20.04+). The bag reader is the path used for the
visualized regression runs. An active display and OpenGL support are needed for
visualization; omit `--visualize` on a headless machine.

## Configurations without a source checkout

The installed package includes the six sensor templates and 29 benchmark YAMLs:

```bash
genz_lio_pipeline list-configs
genz_lio_pipeline export-config default/velodyne.yaml my_robot.yaml
genz_lio_pipeline export-config experiments/geode/vlp16_stairs.yaml stairs.yaml
```

Edit the exported sensor file for your calibration, then pass `--config my_robot.yaml`.
Exports preserve the complete YAML, including sensor topics and publishing settings,
and refuse to overwrite existing files. The packaged originals stay unchanged.

## Run a sequence

```bash
genz_lio_pipeline inspect /data/sequence.bag
genz_lio_pipeline run /data/sequence.bag \
    --config ros/config/default/velodyne.yaml \
    --visualize --output results/my_run
```

For rosbag2, pass the recording directory containing `metadata.yaml` instead
of a `.bag` file. Select a calibrated sensor YAML or a
[benchmark configuration](https://github.com/cocel-postech/genz-lio/blob/master/ros/README.md#benchmark-configurations).
Python paths are relative to the working directory or absolute, rather than
relative to the ROS package. Pass one complete YAML; `--sensor-config` is a
legacy overlay option and is unnecessary for the shipped configurations.
Omitting `--config` uses compiled core defaults, not a sensor YAML.

Topics come from `common.lidar_topic` and `common.imu_topic`. To override them:

```bash
genz_lio_pipeline run /data/sequence.bag \
    --config ros/config/default/velodyne.yaml \
    --lidar-topic /velodyne_points --imu-topic /imu/data \
    --visualize-autoplay --output results/my_run
```

`--visualize-autoplay` starts immediately and closes at the end. Playback runs
as fast as input processing and rendering allow, without a real-time pacing
limit. LiDAR and IMU timestamps still determine estimation timing.

### Ouster PCAP

For a single-sensor PCAP, install the `ouster` or `all` extra and put the sensor's
matching metadata JSON beside the capture with the same filename stem. Native
IMU packets must be present. The reader converts acceleration from g to m/s² and
angular velocity from degrees/s to rad/s, using measurement timestamps in the
sensor clock. Configure the LiDAR/IMU extrinsics for the SDK's sensor coordinates.
Split timestamp resets into separate recordings. This reader has a generated
PCAP regression test; the benchmark evaluations use the rosbag reader.

## Save odometry

Trajectories are saved with or without visualization. The default is
`results/<sequence>_tum.txt`; `--output` selects a directory.

```bash
genz_lio_pipeline run /data/sequence.bag \
    --config ros/config/default/velodyne.yaml --visualize \
    --output results/my_run --format tum --format kitti
```

| Format | File | Row contents |
|---|---|---|
| TUM (default) | `<sequence>_tum.txt` | `timestamp tx ty tz qx qy qz qw` |
| KITTI | `<sequence>_kitti.txt` | First three rows of the 4×4 pose, flattened to 12 values |

The pose maps IMU/body coordinates into the estimator world frame. TUM timestamps
are in seconds and translations in meters; KITTI rows contain no timestamps.
Display camera changes do not affect saved poses. Pressing **Q** or closing the
window normally saves the processed portion. Files are written after the run,
so forcibly killing the process can prevent saving. Reusing the same output
directory and sequence name overwrites those trajectory files.

## Visualizer

The initial camera is **global, top-down**. The retained map, current planar and
non-planar matches, trajectory, and global/body axes are displayed together.

| Key | Action |
|---|---|
| SPACE | Start / pause |
| N | Process the next frame while paused |
| Q | Quit and save the processed trajectory |
| G | Switch local / global view |
| C | Cycle top-down → side → isometric |

The completed map stays open until you quit unless `--visualize-autoplay` is
used. The panel provides point sizes, map opacity, and path width. `Map pts (cm)`
changes rendered point size, not the map's spatial extent. The default map
appearance uses a 6 cm point radius and opacity 0.2.

**GenZ-LIO information** reports planar/non-planar matches, scale, target points,
adaptive voxel size, raw/voxelized points, processing time, FPS, and distance
traveled. Raw points are valid deskewed points before voxelization. FPS is the
inverse of mean odometry computation time, excluding input reading and drawing;
measure whole-run wall time to assess end-to-end throughput. Distance traveled
is accumulated 3D path length, not displacement from the start.

The map display uses retained estimator samples and plane centers. Changed
regions are updated incrementally; it does not append every dense input scan.
There is no display time cutoff, and estimator map pruning is reflected in the
display. Memory can still grow with a larger retained map. The optional CLI
`--visualize-map-spacing 0.2` reduces only displayed map samples; the default
`0` keeps all retained display samples. This setting has no GUI slider and does
not alter odometry, map retention, or the semantic scan points.

### Camera preview

```bash
genz_lio_pipeline run /data/sequence.bag \
    --config ros/config/default/velodyne.yaml --visualize \
    --image-topic /camera/image_raw --output results/my_run
```

The image topic must be inside the input bag. `sensor_msgs/Image` and
`sensor_msgs/CompressedImage` are supported; this is not a live ROS subscription.
Omit `--image-topic` to skip image reading and hide the camera box.

Images are for display only. The preview uses the latest image at or before the
scan end, clears after 0.5 seconds without a recent image, and keeps its aspect
ratio within 640×360 pixels. Missing images do not block estimation. Decoding and
rendering add some work; use whole-run timing to measure its cost on your system.
Pillow is included in the `viz` extra. Camera preview uses Polyscope's inline
image API, with a compatibility path for the Python 3.8 Polyscope 2.5 Linux wheel.

## Python API

For applications that supply synchronized arrays:

```python
from genz_lio import GenZLIO, load_config

lio = GenZLIO(load_config("ros/config/default/velodyne.yaml"))
# Supply points, scan_begin, scan_end, imu, and point_times from your input.
result = lio.register_scan(
    points,                 # (N, 3), LiDAR frame, meters
    scan_begin, scan_end,    # seconds
    imu,                    # (M, 7): t, ax, ay, az, gx, gy, gz
    timestamps=point_times,  # (N,), seconds relative to scan start
)
if result.valid:
    print(result.pose)      # 4×4 world <- IMU/body
```

Provide finite, time-ordered IMU samples up to `scan_end`, with acceleration in
m/s² and angular velocity in rad/s. Optional point attributes must each contain
N entries. Supply per-point timing for deskewing. For raw bag input, the CLI
reader handles timing preparation and synchronization with the shared C++ scan
buffer. Leave the internal `timing_prepared` flag false in direct API calls.
An incomplete final scan may be skipped when no following IMU coverage exists.
Split recordings containing sensor-time rewinds into monotonic runs.

## Rebuild after C++ changes

An editable install updates Python source, but C++ changes require rebuilding
the extension. A catkin/colcon build does not update the Python extension.
From the repository root:

```bash
python -m pip install --upgrade pip
python -m pip install -e '.[rosbag,viz]'
python -c 'from genz_lio import genz_lio_pybind as m; print(m.__file__); print(m.__core_source_hash__)'
python -m pip install pytest
python -m pytest python/tests -q
```

The module path identifies the installation in use; the core source hash helps
detect a stale compiled extension. Run tests from the repository root so imports
resolve to the installed extension. See the
[parameter guide](https://github.com/cocel-postech/genz-lio/blob/master/ros/config/parameter_tuning_guide.md) for tuning and
[ROS troubleshooting](https://github.com/cocel-postech/genz-lio/blob/master/ros/README.md#troubleshooting) for input diagnostics.


## Build distribution files

From the complete repository root:

```bash
python -m pip install --upgrade build twine
python -m build
python -m twine check dist/*
```

This builds a source archive and a local-platform wheel, and validates metadata
without uploading anything. The source archive includes the C++ core, Python
bindings, YAMLs, and license notices. A locally built Linux wheel is not a
portable manylinux wheel. CI separately uses `cibuildwheel` to build and test
Linux x86-64 wheels for CPython 3.8–3.12 and bundle the OpenMP runtime. Other
platforms are not covered by this wheel matrix.

Test a wheel from a fresh virtual environment outside the checkout:

```bash
python -m pip install '/path/to/genz_lio-<version>-<python>-<abi>-<platform>.whl'
genz_lio_pipeline --help
genz_lio_pipeline list-configs
```

Append `[all]` to the quoted wheel path to install the optional dependencies.
PyPI publication is a separate release step; source installation remains the
documented entry point until that release exists.
