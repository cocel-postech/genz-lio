# GenZ-LIO Python

Run recorded LiDAR/IMU data through the C++ estimator, optionally with an
interactive Polyscope visualizer. ROS is not required for offline rosbag reading.

## Installation

The Python package targets Python 3.8–3.12 on Linux x86-64. Source builds require
a C++17 compiler, CMake 3.16+, OpenMP, Eigen 3.4+, and Boost headers.
Visualization needs an active display with OpenGL support.

Install dependencies, get the complete checkout, and create a virtual environment:

```bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev libboost-dev \
    python3-dev python3-venv libgl1
git clone https://github.com/cocel-postech/genz-lio.git
cd genz-lio
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

## Build and install

From the repository root, build the extension and install dependencies for bag
reading and visualization:

```bash
python -m pip install '.[rosbag,viz]'
```

Pip invokes CMake automatically. CMake downloads Eigen 3.4 if the system version
is older; this needs network access during the build. Packaging is configured
by the repository-root `pyproject.toml`; do not run this source install from
`python/`. A separate catkin/colcon build is unnecessary for Python.

## Prepare configurations

The `experiments/` YAMLs provide the sequence-specific configurations for
reproducing the benchmark experiments in the paper. The `default/` YAMLs are
sensor templates for your own recordings.

Export all bundled YAMLs once, preserving their `default/` and `experiments/`
subdirectories under `configs/`:

```bash
genz_lio_pipeline list-configs | while IFS= read -r config; do
    mkdir -p "configs/$(dirname "$config")"
    genz_lio_pipeline export-config "$config" "configs/$config"
done
```

Run this in the directory where you will run the examples below. This copies all
35 YAMLs unchanged and works with both source and wheel installations. Keep this
folder for subsequent runs; there is no need to export again per sequence.
Exported YAMLs are editable. Re-running the export will not overwrite existing
files or your changes.

## Run a sequence

### Example: NarrowWide Handheld-A-01

For the [NarrowWide](https://github.com/cocel-postech/NarrowWide) Handheld-A-01
recording, use the supplied
[`experiments/narrowwide/vlp16_handheld_a_01.yaml`](https://github.com/cocel-postech/genz-lio/blob/master/ros/config/experiments/narrowwide/vlp16_handheld_a_01.yaml)
benchmark configuration from the `configs/` folder prepared above:

```bash
genz_lio_pipeline run "{path_to_bag}/Handheld-A-01.bag" \
    --config configs/experiments/narrowwide/vlp16_handheld_a_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/narrowwide_handheld_a_01
```

Replace `{path_to_bag}` (including the braces) with the directory containing
your NarrowWide `Handheld-A-01.bag` recording. The exported YAML is an unchanged
copy of the experiment configuration, including
its sensor topics and calibration. You do not need to create a configuration
from a default sensor template for this benchmark. The bag must be obtained
separately; it is not bundled with the Python package.

With a source checkout, the equivalent command from the repository root is:

```bash
genz_lio_pipeline run "{path_to_bag}/Handheld-A-01.bag" \
    --config ros/config/experiments/narrowwide/vlp16_handheld_a_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/narrowwide_handheld_a_01
```

The optional `--image-topic /camera/color/image_raw/compressed` displays the
camera stream recorded in Handheld-A-01. Omit this option to hide the camera
panel. Press **SPACE** to start processing.

### Your own sensor

For your own sensor, copy a prepared template, then edit its topics, noise,
timing, and LiDAR-to-IMU calibration for your input:

```bash
cp configs/default/velodyne.yaml my_robot.yaml
```

Inspect the bag and run:

```bash
genz_lio_pipeline inspect /data/sequence.bag
genz_lio_pipeline run /data/sequence.bag \
    --config my_robot.yaml --visualize --output results/my_run
```

Press **SPACE** to start. Omit `--visualize` for headless processing.
For rosbag2, pass the directory containing `metadata.yaml` instead of a `.bag`.
Select a calibrated sensor YAML or a prepared
[benchmark configuration](https://github.com/cocel-postech/genz-lio/blob/master/ros/README.md#benchmark-configurations).
TUM odometry is saved to the selected output directory at completion or normal quit.

Python resolves `--config` as a local file path, relative to the working directory
or absolute. The exported file works with source and wheel installations alike.
Omitting `--config` uses compiled core defaults, not a sensor YAML.

### Camera preview

For example, the Handheld-A-01 bag contains this compressed image topic:

```bash
genz_lio_pipeline run "{path_to_bag}/Handheld-A-01.bag" \
    --config configs/experiments/narrowwide/vlp16_handheld_a_01.yaml --visualize \
    --image-topic /camera/color/image_raw/compressed --output results/narrowwide_handheld_a_01
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

<details>
<summary>Installation extras and the planned PyPI release</summary>

Use `python -m pip install '.[all]'` to include all optional readers and
visualization. Other source extras are `.[rosbag]`, `.[viz]`, and `.[ouster]`;
`python -m pip install .` installs only the core and CLI.

After the package is published on PyPI, installation without a checkout will be:

```bash
python -m pip install 'genz-lio[all]'
```

Use `genz-lio[rosbag,viz]` for bag reading with visualization, or `genz-lio`
for the core and CLI only. A compatible wheel includes the compiled C++ extension;
a source distribution requires the build dependencies above. Until publication,
use the source installation instructions.

The Ouster SDK extra uses version 0.13.1 and requires glibc 2.28 or newer
(for example, Ubuntu 20.04+). The bag reader does not require that SDK.

</details>

## Configurations without a source checkout

Source YAMLs live in `ros/config/`. During packaging, the six sensor templates
and 29 benchmark YAMLs are copied into `genz_lio/configs/` in the installed
package. Installing a wheel does **not** create a `ros/config/` directory in your
working directory and does not require ROS or a cloned repository.

The [preparation step](#prepare-configurations) exports all YAMLs at once. To
list the names or export just one additional copy instead:

```bash
genz_lio_pipeline list-configs
genz_lio_pipeline export-config default/velodyne.yaml another_robot.yaml
```

Edit the exported sensor file for your calibration, then pass `--config another_robot.yaml`.
Exports preserve the complete YAML, including sensor topics and publishing settings,
and refuse to overwrite existing files. The packaged originals stay unchanged.

A name such as `default/velodyne.yaml` is an input to `export-config`, not a
package lookup supported by `--config`. Pass the exported file to `--config`.
With a source checkout, `--config ros/config/default/velodyne.yaml` also works
when run from the repository root. `--sensor-config` is a legacy overlay option;
the shipped YAMLs are complete and do not need it.

## Input options

Topics come from `common.lidar_topic` and `common.imu_topic`. To override them:

```bash
genz_lio_pipeline run /data/sequence.bag \
    --config my_robot.yaml \
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
    --config my_robot.yaml --visualize \
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

## Python API

For applications that supply synchronized arrays:

```python
from genz_lio import GenZLIO, load_config

lio = GenZLIO(load_config("my_robot.yaml"))
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
