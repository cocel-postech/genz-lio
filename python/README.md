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
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-A-01}.bag" \
    --config configs/experiments/narrowwide/vlp16_handheld_a_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/narrowwide_handheld_a_01
```

Use the actual bag directory and filename for the `{path_to_bag}` and
`{NW_Handheld-A-01}` placeholders, respectively. The exported YAML is an unchanged
copy of the experiment configuration, including
its sensor topics and calibration. You do not need to create a configuration
from a default sensor template for this benchmark. The bag must be obtained
separately; it is not bundled with the Python package.

With a source checkout, the equivalent command from the repository root is:

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-A-01}.bag" \
    --config ros/config/experiments/narrowwide/vlp16_handheld_a_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/narrowwide_handheld_a_01
```

The optional `--image-topic /camera/color/image_raw/compressed` displays the
camera stream recorded in Handheld-A-01. Omit this option to hide the camera
panel. Press **SPACE** to start processing.

<details>
<summary>▶️ All benchmark sequences: downloads and Python commands (42 sequences)</summary>

Bag filename labels in braces identify the sequence, not the official download
filename. Substitute the actual downloaded or prepared filename; no file renaming
is required. Labels ending in `_pointcloud` or `_merged` refer to the prepared
inputs described below.

The commands use `configs/experiments/` exported in the preparation step and
write each sequence's trajectory to a separate output directory. All enable
`--visualize`; `--image-topic` is included where the recording contains a camera
stream. Omit that option to disable the preview. Camera streams are display-only.
Download the full recordings with images when using the camera options below.

<details>
<summary>GEODE (7 sequences)</summary>

[Dataset](https://thisparticle.github.io/geode/) · [Download](https://drive.google.com/drive/folders/1hEn3sBAvQhSdUFnGMZCCv-W0Ynj2rWBs).

Use the Alpha (Velodyne) recordings for these YAMLs. The examples select the left camera.

**Stairs**

```bash
genz_lio_pipeline run "{path_to_bag}/{GD_Stairs}.bag" \
    --config configs/experiments/geode/vlp16_stairs.yaml \
    --visualize --image-topic /left_camera/compressed \
    --output results/gd_stairs
```

**Waterways-Short**

```bash
genz_lio_pipeline run "{path_to_bag}/{GD_Waterways-Short}.bag" \
    --config configs/experiments/geode/vlp16_waterways_short.yaml \
    --visualize --image-topic /left_camera/compressed \
    --output results/gd_waterways_short
```

**Waterways-Medium**

```bash
genz_lio_pipeline run "{path_to_bag}/{GD_Waterways-Medium}.bag" \
    --config configs/experiments/geode/vlp16_waterways_medium.yaml \
    --visualize --image-topic /left_camera/compressed \
    --output results/gd_waterways_medium
```

**Waterways-Long**

```bash
genz_lio_pipeline run "{path_to_bag}/{GD_Waterways-Long}.bag" \
    --config configs/experiments/geode/vlp16_waterways_long.yaml \
    --visualize --image-topic /left_camera/compressed \
    --output results/gd_waterways_long
```

**Offroad-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{GD_Offroad-02}.bag" \
    --config configs/experiments/geode/vlp16_offroad.yaml \
    --visualize --image-topic /left_camera/compressed \
    --output results/gd_offroad_02
```

**Offroad-04**

```bash
genz_lio_pipeline run "{path_to_bag}/{GD_Offroad-04}.bag" \
    --config configs/experiments/geode/vlp16_offroad.yaml \
    --visualize --image-topic /left_camera/compressed \
    --output results/gd_offroad_04
```

**Offroad-07**

```bash
genz_lio_pipeline run "{path_to_bag}/{GD_Offroad-07}.bag" \
    --config configs/experiments/geode/vlp16_offroad.yaml \
    --visualize --image-topic /left_camera/compressed \
    --output results/gd_offroad_07
```

</details>

<details>
<summary>ENWIDE (4 sequences)</summary>

[Dataset](https://projects.asl.ethz.ch/datasets/) · [Download](https://doi.org/10.3929/ethz-b-000702477).

These four recordings contain no Image/CompressedImage topic, so the commands omit `--image-topic`.

**Katzensee-S**

```bash
genz_lio_pipeline run "{path_to_bag}/{EW_Katzensee-S}.bag" \
    --config configs/experiments/enwide/os128_enwide.yaml \
    --visualize \
    --output results/ew_katzensee_s
```

**Katzensee-D**

```bash
genz_lio_pipeline run "{path_to_bag}/{EW_Katzensee-D}.bag" \
    --config configs/experiments/enwide/os128_enwide.yaml \
    --visualize \
    --output results/ew_katzensee_d
```

**Intersection-S**

```bash
genz_lio_pipeline run "{path_to_bag}/{EW_Intersection-S}.bag" \
    --config configs/experiments/enwide/os128_enwide.yaml \
    --visualize \
    --output results/ew_intersection_s
```

**Intersection-D**

```bash
genz_lio_pipeline run "{path_to_bag}/{EW_Intersection-D}.bag" \
    --config configs/experiments/enwide/os128_enwide.yaml \
    --visualize \
    --output results/ew_intersection_d
```

</details>

<details>
<summary>NTU VIRAL (3 sequences)</summary>

[Dataset and downloads](https://ntu-aris.github.io/ntu_viral_dataset/).

**SPMS-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{NV_SPMS-01}.bag" \
    --config configs/experiments/ntu_viral/os16_spms_01.yaml \
    --visualize --image-topic /left/image_raw \
    --output results/nv_spms_01
```

**SPMS-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{NV_SPMS-02}.bag" \
    --config configs/experiments/ntu_viral/os16_spms_02.yaml \
    --visualize --image-topic /left/image_raw \
    --output results/nv_spms_02
```

**SPMS-03**

```bash
genz_lio_pipeline run "{path_to_bag}/{NV_SPMS-03}.bag" \
    --config configs/experiments/ntu_viral/os16_spms_03.yaml \
    --visualize --image-topic /left/image_raw \
    --output results/nv_spms_03
```

</details>

<details>
<summary>SuperLoc (4 sequences)</summary>

[Dataset and downloads](https://superodometry.com/superloc).

Download Cave01, Cave02, Cave04, and Corridor02 from the SuperLoc dataset table.

**Cave-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{SL_Cave-01}.bag" \
    --config configs/experiments/superloc/vlp16_cave.yaml \
    --visualize --image-topic /camera_1/image_raw \
    --output results/sl_cave_01
```

**Cave-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{SL_Cave-02}.bag" \
    --config configs/experiments/superloc/vlp16_cave.yaml \
    --visualize --image-topic /camera_1/image_raw \
    --output results/sl_cave_02
```

**Cave-04**

```bash
genz_lio_pipeline run "{path_to_bag}/{SL_Cave-04}.bag" \
    --config configs/experiments/superloc/vlp16_cave.yaml \
    --visualize --image-topic /camera_1/image_raw \
    --output results/sl_cave_04
```

**Corridor-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{SL_Corridor-02}.bag" \
    --config configs/experiments/superloc/vlp16_corridor_02.yaml \
    --visualize --image-topic /camera_1/image_raw \
    --output results/sl_corridor_02
```

</details>

<details>
<summary>NarrowWide (6 sequences)</summary>

[Dataset and downloads](https://github.com/cocel-postech/NarrowWide).

Dataset information and download instructions are maintained in the NarrowWide repository.

**Tracked-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Tracked-01}.bag" \
    --config configs/experiments/narrowwide/mid70_tracked_01.yaml \
    --visualize --image-topic /camera/image_color/compressed \
    --output results/nw_tracked_01
```

**Tracked-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Tracked-02}.bag" \
    --config configs/experiments/narrowwide/mid70_tracked_02.yaml \
    --visualize --image-topic /camera/image_color/compressed \
    --output results/nw_tracked_02
```

**Handheld-A-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-A-01}.bag" \
    --config configs/experiments/narrowwide/vlp16_handheld_a_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/nw_handheld_a_01
```

**Handheld-A-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-A-02}.bag" \
    --config configs/experiments/narrowwide/vlp16_handheld_a_02.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/nw_handheld_a_02
```

**Handheld-B-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-B-01}.bag" \
    --config configs/experiments/narrowwide/avia_handheld_b.yaml \
    --visualize --image-topic /camera/image_color/compressed \
    --output results/nw_handheld_b_01
```

**Handheld-B-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-B-02}.bag" \
    --config configs/experiments/narrowwide/avia_handheld_b.yaml \
    --visualize --image-topic /camera/image_color/compressed \
    --output results/nw_handheld_b_02
```

</details>

<details>
<summary>SubT-MRS (3 sequences)</summary>

[Dataset and downloads](https://superodometry.com/datasets).

**Prepare PointCloud2 input first.** The downloaded bags contain
`/velodyne_packets`, which the Python pipeline cannot decode directly. Convert
those packets with a Velodyne VLP-16 driver to `/velodyne_points`
(`sensor_msgs/PointCloud2`) with per-point timing, while preserving `/imu/data`,
original timestamps, and the image topics shown below. Record the prepared
sequence as one `.bag`; use its filename in the corresponding command below.

Multi-Floor spans `0.bag`–`2.bag`; Laurel-Cavern spans `0.bag`–`10.bag`.
Process all parts in timestamp order into one continuous prepared recording.
The CLI accepts one `.bag` or one rosbag2 recording, not a directory of ROS 1
bag parts. Laurel-Cavern has no image topic; the other two commands require
retaining their camera streams during preparation.

**Long-Corridor**

```bash
genz_lio_pipeline run "{path_to_bag}/{SM_Long-Corridor_pointcloud}.bag" \
    --config configs/experiments/subt_mrs/vlp16_long_corridor.yaml \
    --visualize --image-topic /camera_1/image_raw \
    --output results/sm_long_corridor
```

**Multi-Floor**

```bash
genz_lio_pipeline run "{path_to_bag}/{SM_Multi-Floor_pointcloud}.bag" \
    --config configs/experiments/subt_mrs/vlp16_multi_floor.yaml \
    --visualize --image-topic /cmu_sp1/camera_1/image_raw \
    --output results/sm_multi_floor
```

**Laurel-Cavern**

```bash
genz_lio_pipeline run "{path_to_bag}/{SM_Laurel-Cavern_pointcloud}.bag" \
    --config configs/experiments/subt_mrs/vlp16_laurel_cavern.yaml \
    --visualize \
    --output results/sm_laurel_cavern
```

</details>

<details>
<summary>HILTI 2021 (2 sequences)</summary>

[Dataset and downloads](https://hilti-challenge.com/dataset-2021).

The examples use camera 0 and the MID-70 experiment configurations.

**Basement-04**

```bash
genz_lio_pipeline run "{path_to_bag}/{H21_Basement-04}.bag" \
    --config configs/experiments/hilti21/mid70_hilti21_basement04.yaml \
    --visualize --image-topic /alphasense/cam0/image_raw \
    --output results/h21_basement_04
```

**Drone-Arena**

```bash
genz_lio_pipeline run "{path_to_bag}/{H21_Drone-Arena}.bag" \
    --config configs/experiments/hilti21/mid70_hilti21_drone_arena.yaml \
    --visualize --image-topic /alphasense/cam0/image_raw \
    --output results/h21_drone_arena
```

</details>

<details>
<summary>HILTI 2022 (3 sequences)</summary>

[Dataset and downloads](https://hilti-challenge.com/dataset-2022).

The examples use camera 0 and the Pandar32 experiment configurations.

**Exp-10**

```bash
genz_lio_pipeline run "{path_to_bag}/{H22_Exp-10}.bag" \
    --config configs/experiments/hilti22/pandar32_hilti22_exp10.yaml \
    --visualize --image-topic /alphasense/cam0/image_raw \
    --output results/h22_exp_10
```

**Exp-16**

```bash
genz_lio_pipeline run "{path_to_bag}/{H22_Exp-16}.bag" \
    --config configs/experiments/hilti22/pandar32_hilti22_exp16.yaml \
    --visualize --image-topic /alphasense/cam0/image_raw \
    --output results/h22_exp_16
```

**Exp-18**

```bash
genz_lio_pipeline run "{path_to_bag}/{H22_Exp-18}.bag" \
    --config configs/experiments/hilti22/pandar32_hilti22_exp18.yaml \
    --visualize --image-topic /alphasense/cam0/image_raw \
    --output results/h22_exp_18
```

</details>

<details>
<summary>M3DGR (4 sequences)</summary>

[Dataset and downloads](https://github.com/sjtuyinjie/M3DGR).

The examples select the RGB color camera rather than the compressed depth stream.

**Corridor-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{M3D_Corridor-01}.bag" \
    --config configs/experiments/m3dgr/avia_corridor_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/m3d_corridor_01
```

**Corridor-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{M3D_Corridor-02}.bag" \
    --config configs/experiments/m3dgr/avia_corridor_02.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/m3d_corridor_02
```

**GNSS-denial-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{M3D_GNSS-denial-01}.bag" \
    --config configs/experiments/m3dgr/avia_gnss_denial.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/m3d_gnss_denial_01
```

**GNSS-denial-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{M3D_GNSS-denial-02}.bag" \
    --config configs/experiments/m3dgr/avia_gnss_denial.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/m3d_gnss_denial_02
```

</details>

<details>
<summary>Oxford Spires (6 sequences)</summary>

[Dataset](https://ori-drs.github.io/datasets/oxford-spires/) · [Download](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset).

Download the ROS bags including the camera streams. The examples select
`cam0/debayered/image/compressed`.

**Split recordings:** christ_church-01 and christ_church-02 each contain two
bag parts (`..._0.bag` and `..._1.bag`). Merge both parts in timestamp order,
without changing topics, message contents, or timestamps, into
one merged `.bag` per sequence. The `_merged` placeholders below refer to those files.
The commands below use those prepared files; passing only the first part would
run an incomplete sequence. The other four commands use the original single bags.

**christ_church-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{OS_christ_church-01_merged}.bag" \
    --config configs/experiments/oxford_spires/hesai64_christ_church.yaml \
    --visualize --image-topic /alphasense_driver_ros/cam0/debayered/image/compressed \
    --output results/os_christ_church_01
```

**christ_church-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{OS_christ_church-02_merged}.bag" \
    --config configs/experiments/oxford_spires/hesai64_christ_church.yaml \
    --visualize --image-topic /alphasense_driver_ros/cam0/debayered/image/compressed \
    --output results/os_christ_church_02
```

**christ_church-05**

```bash
genz_lio_pipeline run "{path_to_bag}/{OS_christ_church-05}.bag" \
    --config configs/experiments/oxford_spires/hesai64_christ_church.yaml \
    --visualize --image-topic /alphasense_driver_ros/cam0/debayered/image/compressed \
    --output results/os_christ_church_05
```

**blenheim_palace-01**

```bash
genz_lio_pipeline run "{path_to_bag}/{OS_blenheim_palace-01}.bag" \
    --config configs/experiments/oxford_spires/hesai64_blenheim_palace.yaml \
    --visualize --image-topic /alphasense_driver_ros/cam0/debayered/image/compressed \
    --output results/os_blenheim_palace_01
```

**blenheim_palace-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{OS_blenheim_palace-02}.bag" \
    --config configs/experiments/oxford_spires/hesai64_blenheim_palace.yaml \
    --visualize --image-topic /alphasense_driver_ros/cam0/debayered/image/compressed \
    --output results/os_blenheim_palace_02
```

**blenheim_palace-05**

```bash
genz_lio_pipeline run "{path_to_bag}/{OS_blenheim_palace-05}.bag" \
    --config configs/experiments/oxford_spires/hesai64_blenheim_palace.yaml \
    --visualize --image-topic /alphasense_driver_ros/cam0/debayered/image/compressed \
    --output results/os_blenheim_palace_05
```

</details>

</details>

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
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-A-01}.bag" \
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
