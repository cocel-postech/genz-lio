<div align="center">

# GenZ-LIO

**Generalizable LiDAR-Inertial Odometry Beyond Confined–Open Boundaries**

[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](cpp/genz_lio)
[![Python](https://img.shields.io/badge/Python-3.8--3.12-yellow)](python/README.md)
[![ROS 1](https://img.shields.io/badge/ROS%201-Noetic-green)](ros/README.md)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-orange)](ros/README.md)
[![License](https://img.shields.io/badge/License-GPL%20v2-red.svg)](LICENSE)

[Demo](https://www.youtube.com/watch?v=EyTJbdC_AA4)
<span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
[Paper](https://arxiv.org/abs/2603.16273)
<span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
[Dataset](https://github.com/cocel-postech/NarrowWide)
<span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
[Install](#python-support)
<span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
[Python](python/README.md)
<span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
[ROS](ros/README.md)

<a href="pictures/GenZ-LIO.gif" title="Open the 30-second GIF">
  <img src="pictures/GenZ-LIO_20s.webp" width="1000" alt="GenZ-LIO on NarrowWide Handheld-A-01" />
</a>
<br />
<br />

</div>

[GenZ-LIO](https://arxiv.org/abs/2603.16273) is designed for robust and computationally efficient LiDAR-inertial odometry across confined spaces, open environments, and transitions between them.

## Python support

Process recorded data with an optional visualizer; no ROS installation is needed.

<details>
<summary>Install and run (Linux x86-64, Python 3.8–3.12)</summary>

### 1. Prepare the environment

Use Linux x86-64 with Python 3.8–3.12. Ubuntu 20.04, 22.04, and 24.04 provide
Python 3.8, 3.10, and 3.12, respectively.

```bash
sudo apt-get update
sudo apt-get install -y python3-venv libgl1
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

### 2. Install

Install all optional readers (rosbag and Ouster PCAP) and the visualizer:

```bash
python -m pip install 'genz-lio[all]'
```

The published wheels include the compiled C++ extension; no local C++ build or
ROS installation is required. Plain `pip install genz-lio` installs the core and
CLI.

### 3. Prepare configurations

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
36 YAMLs unchanged and works with both source and wheel installations. Keep this
folder for subsequent runs; there is no need to export again per sequence.
Exported YAMLs are editable. Re-running the export will not overwrite existing
files or your changes.

### 4. Run

**1) For reproducing benchmark experiments (e.g., NarrowWide Handheld-A-01)**

Use the experiment YAML prepared above with the
[NarrowWide](https://github.com/cocel-postech/NarrowWide) Handheld-A-01 recording:

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-A-01}.bag" \
    --config configs/experiments/narrowwide/vlp16_handheld_a_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/narrowwide_handheld_a_01
```

The experiment YAML already contains the benchmark sensor
topics and calibration. Obtain the dataset separately.
The optional `--image-topic` displays the camera images recorded in this bag;
omit it to run without the camera panel.

Press **SPACE** to start. For rosbag2, pass the recording directory instead of a
`.bag` file. The exported YAMLs also work outside the source checkout.

<details>
<summary>▶️ All benchmark sequences: downloads and Python commands (42 sequences)</summary>

The commands use `configs/experiments/` exported in the preparation step and
write each sequence's trajectory to a separate output directory. All enable
`--visualize`; `--image-topic` is included where the recording contains a camera
stream. Omit that option to disable the preview. Camera streams are display-only.
Download the full recordings with images when using the camera options below.

Bag filename labels in braces identify the sequence, not the official download
filename. Substitute the actual downloaded or prepared filename; no file renaming
is required. Labels ending in `_pointcloud` or `_merged` refer to the prepared
inputs described below.

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
    --config configs/experiments/narrowwide/avia_handheld_b_01.yaml \
    --visualize --image-topic /camera/image_color/compressed \
    --output results/nw_handheld_b_01
```

**Handheld-B-02**

```bash
genz_lio_pipeline run "{path_to_bag}/{NW_Handheld-B-02}.bag" \
    --config configs/experiments/narrowwide/avia_handheld_b_02.yaml \
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

**2) For your own sensor, copy a template, calibrate it, then open the visualizer:**

If you use a Velodyne LiDAR, start with the following template:

```bash
cp configs/default/velodyne.yaml my_robot.yaml
# Edit my_robot.yaml for your sensor topics and LiDAR-to-IMU calibration.
genz_lio_pipeline run /data/sequence.bag \
    --config my_robot.yaml --visualize --output results/my_run
```

To show camera images from your own recording, add `--image-topic` followed by
an Image or CompressedImage topic present in that bag.

Parameter tuning guidance is available in the [parameter guide](https://github.com/cocel-postech/genz-lio/blob/master/ros/config/parameter_tuning_guide.md).

See [python/README.md](python/README.md) for rebuilding after C++ changes,
saving odometry, input options, camera input requirements, the Python API,
and Ouster PCAP input.

</details>

## ROS 1 support

<details>
<summary>Install, build, and run with RViz (Noetic)</summary>

### 1. Install dependencies

Start with [ROS 1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) installed
on Ubuntu 20.04:

```bash
source /opt/ros/noetic/setup.bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev libboost-dev \
    libyaml-cpp-dev ros-noetic-roscpp ros-noetic-rospy ros-noetic-roslib \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf \
    ros-noetic-tf2-ros ros-noetic-visualization-msgs ros-noetic-rviz \
    ros-noetic-rosbag
```

For Livox `CustomMsg` input, complete the optional driver build below before
building GenZ-LIO. PointCloud2 input does not require this step.

<a id="livox-ros-1"></a>
<details>
<summary>Livox ROS 1: build CustomMsg support (optional)</summary>

For `livox_ros_driver/CustomMsg`, install
[Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
following its build instructions, then build
[livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) in a separate workspace:

```bash
source /opt/ros/noetic/setup.bash
git clone https://github.com/Livox-SDK/livox_ros_driver.git ~/livox_ws/src
cd ~/livox_ws
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Keep this shell for the next step. GenZ-LIO's ROS 1 wrapper expects
`livox_ros_driver`, not the ROS 1 variant of `livox_ros_driver2`.

</details>

### 2. Build GenZ-LIO

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/cocel-postech/genz-lio.git
cd ~/catkin_ws
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

The build commands limit compilation to two parallel jobs to reduce peak memory
use. This does not change `runtime.max_threads` during odometry.

If using Livox CustomMsg, check that CMake reports
`Livox CustomMsg support enabled`. If the driver was added after GenZ-LIO was
built, source its workspace and rerun
`catkin_make -j2 --force-cmake -DCMAKE_BUILD_TYPE=Release` from your GenZ-LIO workspace.

### 3. Run

Use the package's `config/experiments/` YAMLs for the paper's benchmark sequences,
or a calibrated `config/default/` template for your own sensor. `config:=` accepts
a path relative to the package's `config/` directory, or an absolute YAML path.

**1) For reproducing benchmark experiments (e.g., NarrowWide Handheld-A-01)**

Start GenZ-LIO with RViz in the first terminal:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch genz_lio odometry.launch \
    config:=experiments/narrowwide/vlp16_handheld_a_01.yaml
```

Wait for the node to start, then play the recording in a second terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rosbag play --delay 1 "{path_to_bag}/{NW_Handheld-A-01}.bag"
```

`--delay 1` gives subscribers time to connect after each topic is advertised,
helping preserve the first IMU samples used for initialization. Increase it if
startup connections are slow.

Replace the bag placeholders with the actual downloaded or prepared file.
GenZ-LIO consumes PointCloud2 or supported Livox CustomMsg plus IMU messages;
raw Velodyne packets must first be decoded to PointCloud2 with point timing.

<details>
<summary>▶️ All benchmark sequences: downloads and ROS 1 configurations (42 sequences)</summary>

Use the YAML path below as the `config:=` argument in the launch command above.
Paths are relative to the package's `config/` directory.

<details>
<summary>GEODE (7 sequences)</summary>

[Dataset](https://thisparticle.github.io/geode/) · [Download](https://drive.google.com/drive/folders/1hEn3sBAvQhSdUFnGMZCCv-W0Ynj2rWBs).

Use the Alpha (Velodyne) recordings for these YAMLs.

| Sequence | YAML (`config:=`) |
|---|---|
| Stairs | [experiments/geode/vlp16_stairs.yaml](ros/config/experiments/geode/vlp16_stairs.yaml) |
| Waterways-Short | [experiments/geode/vlp16_waterways_short.yaml](ros/config/experiments/geode/vlp16_waterways_short.yaml) |
| Waterways-Medium | [experiments/geode/vlp16_waterways_medium.yaml](ros/config/experiments/geode/vlp16_waterways_medium.yaml) |
| Waterways-Long | [experiments/geode/vlp16_waterways_long.yaml](ros/config/experiments/geode/vlp16_waterways_long.yaml) |
| Offroad-02 | [experiments/geode/vlp16_offroad.yaml](ros/config/experiments/geode/vlp16_offroad.yaml) |
| Offroad-04 | [experiments/geode/vlp16_offroad.yaml](ros/config/experiments/geode/vlp16_offroad.yaml) |
| Offroad-07 | [experiments/geode/vlp16_offroad.yaml](ros/config/experiments/geode/vlp16_offroad.yaml) |

</details>

<details>
<summary>ENWIDE (4 sequences)</summary>

[Dataset](https://projects.asl.ethz.ch/datasets/) · [Download](https://doi.org/10.3929/ethz-b-000702477).

| Sequence | YAML (`config:=`) |
|---|---|
| Katzensee-S | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |
| Katzensee-D | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |
| Intersection-S | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |
| Intersection-D | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |

</details>

<details>
<summary>NTU VIRAL (3 sequences)</summary>

[Dataset and downloads](https://ntu-aris.github.io/ntu_viral_dataset/).

| Sequence | YAML (`config:=`) |
|---|---|
| SPMS-01 | [experiments/ntu_viral/os16_spms_01.yaml](ros/config/experiments/ntu_viral/os16_spms_01.yaml) |
| SPMS-02 | [experiments/ntu_viral/os16_spms_02.yaml](ros/config/experiments/ntu_viral/os16_spms_02.yaml) |
| SPMS-03 | [experiments/ntu_viral/os16_spms_03.yaml](ros/config/experiments/ntu_viral/os16_spms_03.yaml) |

</details>

<details>
<summary>SuperLoc (4 sequences)</summary>

[Dataset and downloads](https://superodometry.com/superloc).

Download Cave01, Cave02, Cave04, and Corridor02 from the SuperLoc dataset table.

| Sequence | YAML (`config:=`) |
|---|---|
| Cave-01 | [experiments/superloc/vlp16_cave.yaml](ros/config/experiments/superloc/vlp16_cave.yaml) |
| Cave-02 | [experiments/superloc/vlp16_cave.yaml](ros/config/experiments/superloc/vlp16_cave.yaml) |
| Cave-04 | [experiments/superloc/vlp16_cave.yaml](ros/config/experiments/superloc/vlp16_cave.yaml) |
| Corridor-02 | [experiments/superloc/vlp16_corridor_02.yaml](ros/config/experiments/superloc/vlp16_corridor_02.yaml) |

</details>

<details>
<summary>NarrowWide (6 sequences)</summary>

[Dataset and downloads](https://github.com/cocel-postech/NarrowWide).

Dataset information and download instructions are maintained in the NarrowWide repository.

| Sequence | YAML (`config:=`) |
|---|---|
| Tracked-01 | [experiments/narrowwide/mid70_tracked_01.yaml](ros/config/experiments/narrowwide/mid70_tracked_01.yaml) |
| Tracked-02 | [experiments/narrowwide/mid70_tracked_02.yaml](ros/config/experiments/narrowwide/mid70_tracked_02.yaml) |
| Handheld-A-01 | [experiments/narrowwide/vlp16_handheld_a_01.yaml](ros/config/experiments/narrowwide/vlp16_handheld_a_01.yaml) |
| Handheld-A-02 | [experiments/narrowwide/vlp16_handheld_a_02.yaml](ros/config/experiments/narrowwide/vlp16_handheld_a_02.yaml) |
| Handheld-B-01 | [experiments/narrowwide/avia_handheld_b_01.yaml](ros/config/experiments/narrowwide/avia_handheld_b_01.yaml) |
| Handheld-B-02 | [experiments/narrowwide/avia_handheld_b_02.yaml](ros/config/experiments/narrowwide/avia_handheld_b_02.yaml) |

</details>

<details>
<summary>SubT-MRS (3 sequences)</summary>

[Dataset and downloads](https://superodometry.com/datasets).

**Prepare PointCloud2 input first.** Decode `/velodyne_packets` with a
Velodyne VLP-16 driver to `/velodyne_points` (`sensor_msgs/PointCloud2`) with
per-point timing, preserving `/imu/data` and original timestamps. GenZ-LIO
does not consume raw packets directly.

Multi-Floor spans `0.bag`–`2.bag`; Laurel-Cavern spans `0.bag`–`10.bag`.
Process all parts in timestamp order into one continuous prepared recording.
Use one prepared `.bag` per sequence for the playback example above.

| Sequence | YAML (`config:=`) |
|---|---|
| Long-Corridor | [experiments/subt_mrs/vlp16_long_corridor.yaml](ros/config/experiments/subt_mrs/vlp16_long_corridor.yaml) |
| Multi-Floor | [experiments/subt_mrs/vlp16_multi_floor.yaml](ros/config/experiments/subt_mrs/vlp16_multi_floor.yaml) |
| Laurel-Cavern | [experiments/subt_mrs/vlp16_laurel_cavern.yaml](ros/config/experiments/subt_mrs/vlp16_laurel_cavern.yaml) |

</details>

<details>
<summary>HILTI 2021 (2 sequences)</summary>

[Dataset and downloads](https://hilti-challenge.com/dataset-2021).

Use the MID-70 recordings with these experiment configurations.

| Sequence | YAML (`config:=`) |
|---|---|
| Basement-04 | [experiments/hilti21/mid70_hilti21_basement04.yaml](ros/config/experiments/hilti21/mid70_hilti21_basement04.yaml) |
| Drone-Arena | [experiments/hilti21/mid70_hilti21_drone_arena.yaml](ros/config/experiments/hilti21/mid70_hilti21_drone_arena.yaml) |

</details>

<details>
<summary>HILTI 2022 (3 sequences)</summary>

[Dataset and downloads](https://hilti-challenge.com/dataset-2022).

Use the Pandar32 recordings with these experiment configurations.

| Sequence | YAML (`config:=`) |
|---|---|
| Exp-10 | [experiments/hilti22/pandar32_hilti22_exp10.yaml](ros/config/experiments/hilti22/pandar32_hilti22_exp10.yaml) |
| Exp-16 | [experiments/hilti22/pandar32_hilti22_exp16.yaml](ros/config/experiments/hilti22/pandar32_hilti22_exp16.yaml) |
| Exp-18 | [experiments/hilti22/pandar32_hilti22_exp18.yaml](ros/config/experiments/hilti22/pandar32_hilti22_exp18.yaml) |

</details>

<details>
<summary>M3DGR (4 sequences)</summary>

[Dataset and downloads](https://github.com/sjtuyinjie/M3DGR).

| Sequence | YAML (`config:=`) |
|---|---|
| Corridor-01 | [experiments/m3dgr/avia_corridor_01.yaml](ros/config/experiments/m3dgr/avia_corridor_01.yaml) |
| Corridor-02 | [experiments/m3dgr/avia_corridor_02.yaml](ros/config/experiments/m3dgr/avia_corridor_02.yaml) |
| GNSS-denial-01 | [experiments/m3dgr/avia_gnss_denial.yaml](ros/config/experiments/m3dgr/avia_gnss_denial.yaml) |
| GNSS-denial-02 | [experiments/m3dgr/avia_gnss_denial.yaml](ros/config/experiments/m3dgr/avia_gnss_denial.yaml) |

</details>

<details>
<summary>Oxford Spires (6 sequences)</summary>

[Dataset](https://ori-drs.github.io/datasets/oxford-spires/) · [Download](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset).

**Split recordings:** christ_church-01 and christ_church-02 each contain two
bag parts (`..._0.bag` and `..._1.bag`). Include both in timestamp order,
preserving topics, message contents, and timestamps. Using only the first part
runs an incomplete sequence. The other four sequences have one source bag each.
ROS 1 can play both parts together without merging. Start GenZ-LIO with the
matching YAML, then run this in the playback terminal:

```bash
rosbag play --delay 1 "{path_to_bag}/{sequence}_0.bag" "{path_to_bag}/{sequence}_1.bag"
```

Replace the placeholders with the actual filenames; both parts are replayed
in timestamp order.

| Sequence | YAML (`config:=`) |
|---|---|
| christ_church-01 | [experiments/oxford_spires/hesai64_christ_church.yaml](ros/config/experiments/oxford_spires/hesai64_christ_church.yaml) |
| christ_church-02 | [experiments/oxford_spires/hesai64_christ_church.yaml](ros/config/experiments/oxford_spires/hesai64_christ_church.yaml) |
| christ_church-05 | [experiments/oxford_spires/hesai64_christ_church.yaml](ros/config/experiments/oxford_spires/hesai64_christ_church.yaml) |
| blenheim_palace-01 | [experiments/oxford_spires/hesai64_blenheim_palace.yaml](ros/config/experiments/oxford_spires/hesai64_blenheim_palace.yaml) |
| blenheim_palace-02 | [experiments/oxford_spires/hesai64_blenheim_palace.yaml](ros/config/experiments/oxford_spires/hesai64_blenheim_palace.yaml) |
| blenheim_palace-05 | [experiments/oxford_spires/hesai64_blenheim_palace.yaml](ros/config/experiments/oxford_spires/hesai64_blenheim_palace.yaml) |

</details>

</details>

**2) For your own sensor, copy a template, calibrate it, then open RViz:**

If you use a Velodyne LiDAR, start with the following template:

```bash
cp ~/catkin_ws/src/genz-lio/ros/config/default/velodyne.yaml ~/catkin_ws/src/genz-lio/ros/config/my_robot.yaml
```

Edit `my_robot.yaml` for your topics, sensor type, per-point timing, noise, and
LiDAR-to-IMU calibration. The default extrinsics are placeholders. Then launch:

```bash
roslaunch genz_lio odometry.launch config:=my_robot.yaml
```

In the second terminal, play your own recording instead of the benchmark above.
For live input, start your LiDAR/IMU drivers instead of the bag player.
The ROS node publishes `/Odometry`; it does not automatically save a trajectory.

Parameter tuning guidance is available in the [parameter guide](ros/config/parameter_tuning_guide.md).

See [ros/README.md](ros/README.md#additional-options) for TUM/KITTI odometry saving,
ROS 1-to-ROS 2 bag conversion, published topics, and troubleshooting.

</details>

## ROS 2 support

<details>
<summary>Install, build, and run with RViz (Humble / Jazzy)</summary>

### 1. Install dependencies

Start with ROS 2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
on Ubuntu 22.04 or [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
on Ubuntu 24.04. Use a separate shell and workspace for each distribution.

```bash
# ROS 2: use jazzy instead of humble on Ubuntu 24.04.
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev libboost-dev \
    libyaml-cpp-dev libpcl-dev python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-rclpy ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-launch-ros ros-${ROS_DISTRO}-rosbag2 ros-${ROS_DISTRO}-rviz2
```

For Livox `CustomMsg` input, complete the optional driver build below before
building GenZ-LIO. PointCloud2 input does not require this step.

<a id="livox-ros-2"></a>
<details>
<summary>Livox ROS 2: build CustomMsg support (optional)</summary>

For `livox_ros_driver2/msg/CustomMsg`, install
[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
following its build instructions, then build
[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) in a dedicated
workspace. Its build script clears that workspace's build/install directories,
so keep it separate from GenZ-LIO and use a different workspace per ROS distribution.

In the same Humble or Jazzy shell used above:

```bash
mkdir -p ~/livox_ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/livox_ros2_ws/src/livox_ros_driver2
cd ~/livox_ros2_ws/src/livox_ros_driver2
MAKEFLAGS="-j2" bash build.sh "$ROS_DISTRO"
source ~/livox_ros2_ws/install/setup.bash
```

The driver build accepts `humble` or `jazzy`. Keep this shell for the next step.
For bag replay, the recorded CustomMsg type must match the ROS 2 driver's type;
renaming a topic does not convert ROS 1 messages into ROS 2 messages.

</details>

### 2. Build GenZ-LIO

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/cocel-postech/genz-lio.git
cd ~/ros2_ws
MAKEFLAGS="-j2" colcon build --packages-select genz_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`MAKEFLAGS="-j2"` limits compilation to two parallel jobs to reduce peak memory
use. This does not change `runtime.max_threads` during odometry.

If using Livox CustomMsg, check `log/latest_build/genz_lio/stdout.log` for
`Livox CustomMsg support enabled`. If the driver was added after GenZ-LIO was
built, source its workspace and force CMake to detect it:

```bash
cd ~/ros2_ws
MAKEFLAGS="-j2" colcon build --packages-select genz_lio --cmake-force-configure \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 3. Run

Use the package's `config/experiments/` YAMLs for the paper's benchmark sequences,
or a calibrated `config/default/` template for your own sensor. `config:=` accepts
a path relative to the package's `config/` directory, or an absolute YAML path.

**1) For reproducing benchmark experiments (e.g., NarrowWide Handheld-A-01)**

Start GenZ-LIO with RViz in the first terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch genz_lio odometry.launch.py \
    config:=experiments/narrowwide/vlp16_handheld_a_01.yaml
```

Wait for the node to start, then play the recording in a second terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play --delay 1 "{path_to_rosbag2_recording}"
```

`--delay 1` allows discovery before playback starts, helping preserve the initial
sensor messages. Increase it if discovery is slow.

Use a rosbag2 recording of Handheld-A-01 with the original sensor topics,
timestamps, and message fields. The command above expects a rosbag2 directory,
not a ROS 1 `.bag`; follow the [bag conversion guide](ros/README.md#convert-ros-1-bags-to-ros-2) if needed.
Source `install/setup.bash` in every terminal, including the bag player and
drivers. With Fast DDS, this also applies the package's default transport profile
unless you have already selected your own profile.
GenZ-LIO consumes PointCloud2 or supported Livox CustomMsg plus IMU messages;
raw Velodyne packets must first be decoded to PointCloud2 with point timing.

<details>
<summary>▶️ All benchmark sequences: downloads and ROS 2 configurations (42 sequences)</summary>

Use the YAML path below as the `config:=` argument in the launch command above.
Paths are relative to the package's `config/` directory.

Downloads may contain ROS 1 bags. Prepare rosbag2 recordings for ROS 2 playback,
preserving sensor topics, timestamps, per-point timing, and IMU data. Livox
CustomMsg recordings must use `livox_ros_driver2/msg/CustomMsg`; renaming the
topic alone does not convert the message type.

<details>
<summary>GEODE (7 sequences)</summary>

[Dataset](https://thisparticle.github.io/geode/) · [Download](https://drive.google.com/drive/folders/1hEn3sBAvQhSdUFnGMZCCv-W0Ynj2rWBs).

Use the Alpha (Velodyne) recordings for these YAMLs.

| Sequence | YAML (`config:=`) |
|---|---|
| Stairs | [experiments/geode/vlp16_stairs.yaml](ros/config/experiments/geode/vlp16_stairs.yaml) |
| Waterways-Short | [experiments/geode/vlp16_waterways_short.yaml](ros/config/experiments/geode/vlp16_waterways_short.yaml) |
| Waterways-Medium | [experiments/geode/vlp16_waterways_medium.yaml](ros/config/experiments/geode/vlp16_waterways_medium.yaml) |
| Waterways-Long | [experiments/geode/vlp16_waterways_long.yaml](ros/config/experiments/geode/vlp16_waterways_long.yaml) |
| Offroad-02 | [experiments/geode/vlp16_offroad.yaml](ros/config/experiments/geode/vlp16_offroad.yaml) |
| Offroad-04 | [experiments/geode/vlp16_offroad.yaml](ros/config/experiments/geode/vlp16_offroad.yaml) |
| Offroad-07 | [experiments/geode/vlp16_offroad.yaml](ros/config/experiments/geode/vlp16_offroad.yaml) |

</details>

<details>
<summary>ENWIDE (4 sequences)</summary>

[Dataset](https://projects.asl.ethz.ch/datasets/) · [Download](https://doi.org/10.3929/ethz-b-000702477).

| Sequence | YAML (`config:=`) |
|---|---|
| Katzensee-S | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |
| Katzensee-D | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |
| Intersection-S | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |
| Intersection-D | [experiments/enwide/os128_enwide.yaml](ros/config/experiments/enwide/os128_enwide.yaml) |

</details>

<details>
<summary>NTU VIRAL (3 sequences)</summary>

[Dataset and downloads](https://ntu-aris.github.io/ntu_viral_dataset/).

| Sequence | YAML (`config:=`) |
|---|---|
| SPMS-01 | [experiments/ntu_viral/os16_spms_01.yaml](ros/config/experiments/ntu_viral/os16_spms_01.yaml) |
| SPMS-02 | [experiments/ntu_viral/os16_spms_02.yaml](ros/config/experiments/ntu_viral/os16_spms_02.yaml) |
| SPMS-03 | [experiments/ntu_viral/os16_spms_03.yaml](ros/config/experiments/ntu_viral/os16_spms_03.yaml) |

</details>

<details>
<summary>SuperLoc (4 sequences)</summary>

[Dataset and downloads](https://superodometry.com/superloc).

Download Cave01, Cave02, Cave04, and Corridor02 from the SuperLoc dataset table.

| Sequence | YAML (`config:=`) |
|---|---|
| Cave-01 | [experiments/superloc/vlp16_cave.yaml](ros/config/experiments/superloc/vlp16_cave.yaml) |
| Cave-02 | [experiments/superloc/vlp16_cave.yaml](ros/config/experiments/superloc/vlp16_cave.yaml) |
| Cave-04 | [experiments/superloc/vlp16_cave.yaml](ros/config/experiments/superloc/vlp16_cave.yaml) |
| Corridor-02 | [experiments/superloc/vlp16_corridor_02.yaml](ros/config/experiments/superloc/vlp16_corridor_02.yaml) |

</details>

<details>
<summary>NarrowWide (6 sequences)</summary>

[Dataset and downloads](https://github.com/cocel-postech/NarrowWide).

Dataset information and download instructions are maintained in the NarrowWide repository.

| Sequence | YAML (`config:=`) |
|---|---|
| Tracked-01 | [experiments/narrowwide/mid70_tracked_01.yaml](ros/config/experiments/narrowwide/mid70_tracked_01.yaml) |
| Tracked-02 | [experiments/narrowwide/mid70_tracked_02.yaml](ros/config/experiments/narrowwide/mid70_tracked_02.yaml) |
| Handheld-A-01 | [experiments/narrowwide/vlp16_handheld_a_01.yaml](ros/config/experiments/narrowwide/vlp16_handheld_a_01.yaml) |
| Handheld-A-02 | [experiments/narrowwide/vlp16_handheld_a_02.yaml](ros/config/experiments/narrowwide/vlp16_handheld_a_02.yaml) |
| Handheld-B-01 | [experiments/narrowwide/avia_handheld_b_01.yaml](ros/config/experiments/narrowwide/avia_handheld_b_01.yaml) |
| Handheld-B-02 | [experiments/narrowwide/avia_handheld_b_02.yaml](ros/config/experiments/narrowwide/avia_handheld_b_02.yaml) |

</details>

<details>
<summary>SubT-MRS (3 sequences)</summary>

[Dataset and downloads](https://superodometry.com/datasets).

**Prepare PointCloud2 input first.** Decode `/velodyne_packets` with a
Velodyne VLP-16 driver to `/velodyne_points` (`sensor_msgs/PointCloud2`) with
per-point timing, preserving `/imu/data` and original timestamps. GenZ-LIO
does not consume raw packets directly.

Multi-Floor spans `0.bag`–`2.bag`; Laurel-Cavern spans `0.bag`–`10.bag`.
Process all parts in timestamp order into one continuous prepared recording.
Record or convert the prepared data to one rosbag2 recording per sequence.

| Sequence | YAML (`config:=`) |
|---|---|
| Long-Corridor | [experiments/subt_mrs/vlp16_long_corridor.yaml](ros/config/experiments/subt_mrs/vlp16_long_corridor.yaml) |
| Multi-Floor | [experiments/subt_mrs/vlp16_multi_floor.yaml](ros/config/experiments/subt_mrs/vlp16_multi_floor.yaml) |
| Laurel-Cavern | [experiments/subt_mrs/vlp16_laurel_cavern.yaml](ros/config/experiments/subt_mrs/vlp16_laurel_cavern.yaml) |

</details>

<details>
<summary>HILTI 2021 (2 sequences)</summary>

[Dataset and downloads](https://hilti-challenge.com/dataset-2021).

Use the MID-70 recordings with these experiment configurations.

| Sequence | YAML (`config:=`) |
|---|---|
| Basement-04 | [experiments/hilti21/mid70_hilti21_basement04.yaml](ros/config/experiments/hilti21/mid70_hilti21_basement04.yaml) |
| Drone-Arena | [experiments/hilti21/mid70_hilti21_drone_arena.yaml](ros/config/experiments/hilti21/mid70_hilti21_drone_arena.yaml) |

</details>

<details>
<summary>HILTI 2022 (3 sequences)</summary>

[Dataset and downloads](https://hilti-challenge.com/dataset-2022).

Use the Pandar32 recordings with these experiment configurations.

| Sequence | YAML (`config:=`) |
|---|---|
| Exp-10 | [experiments/hilti22/pandar32_hilti22_exp10.yaml](ros/config/experiments/hilti22/pandar32_hilti22_exp10.yaml) |
| Exp-16 | [experiments/hilti22/pandar32_hilti22_exp16.yaml](ros/config/experiments/hilti22/pandar32_hilti22_exp16.yaml) |
| Exp-18 | [experiments/hilti22/pandar32_hilti22_exp18.yaml](ros/config/experiments/hilti22/pandar32_hilti22_exp18.yaml) |

</details>

<details>
<summary>M3DGR (4 sequences)</summary>

[Dataset and downloads](https://github.com/sjtuyinjie/M3DGR).

| Sequence | YAML (`config:=`) |
|---|---|
| Corridor-01 | [experiments/m3dgr/avia_corridor_01.yaml](ros/config/experiments/m3dgr/avia_corridor_01.yaml) |
| Corridor-02 | [experiments/m3dgr/avia_corridor_02.yaml](ros/config/experiments/m3dgr/avia_corridor_02.yaml) |
| GNSS-denial-01 | [experiments/m3dgr/avia_gnss_denial.yaml](ros/config/experiments/m3dgr/avia_gnss_denial.yaml) |
| GNSS-denial-02 | [experiments/m3dgr/avia_gnss_denial.yaml](ros/config/experiments/m3dgr/avia_gnss_denial.yaml) |

</details>

<details>
<summary>Oxford Spires (6 sequences)</summary>

[Dataset](https://ori-drs.github.io/datasets/oxford-spires/) · [Download](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset).

**Split recordings:** christ_church-01 and christ_church-02 each contain two
bag parts (`..._0.bag` and `..._1.bag`). Include both in timestamp order,
preserving topics, message contents, and timestamps. Using only the first part
runs an incomplete sequence. The other four sequences have one source bag each.
Prepare one continuous rosbag2 recording containing both parts per sequence.

| Sequence | YAML (`config:=`) |
|---|---|
| christ_church-01 | [experiments/oxford_spires/hesai64_christ_church.yaml](ros/config/experiments/oxford_spires/hesai64_christ_church.yaml) |
| christ_church-02 | [experiments/oxford_spires/hesai64_christ_church.yaml](ros/config/experiments/oxford_spires/hesai64_christ_church.yaml) |
| christ_church-05 | [experiments/oxford_spires/hesai64_christ_church.yaml](ros/config/experiments/oxford_spires/hesai64_christ_church.yaml) |
| blenheim_palace-01 | [experiments/oxford_spires/hesai64_blenheim_palace.yaml](ros/config/experiments/oxford_spires/hesai64_blenheim_palace.yaml) |
| blenheim_palace-02 | [experiments/oxford_spires/hesai64_blenheim_palace.yaml](ros/config/experiments/oxford_spires/hesai64_blenheim_palace.yaml) |
| blenheim_palace-05 | [experiments/oxford_spires/hesai64_blenheim_palace.yaml](ros/config/experiments/oxford_spires/hesai64_blenheim_palace.yaml) |

</details>

</details>

**2) For your own sensor, copy a template, calibrate it, then open RViz:**

If you use a Velodyne LiDAR, start with the following template:

```bash
cp ~/ros2_ws/src/genz-lio/ros/config/default/velodyne.yaml ~/my_robot.yaml
```

Edit `my_robot.yaml` for your topics, sensor type, per-point timing, noise, and
LiDAR-to-IMU calibration. The default extrinsics are placeholders. Then launch:

```bash
ros2 launch genz_lio odometry.launch.py config:="$HOME/my_robot.yaml"
```

In the second terminal, play your own recording instead of the benchmark above.
For live input, start your LiDAR/IMU drivers instead of the bag player.
The ROS node publishes `/Odometry`; it does not automatically save a trajectory.

Parameter tuning guidance is available in the [parameter guide](ros/config/parameter_tuning_guide.md).

See [ros/README.md](ros/README.md#additional-options) for TUM/KITTI odometry saving,
ROS 1-to-ROS 2 bag conversion, troubleshooting, and DDS settings.

</details>

## :pencil: Citation

If you use GenZ-LIO, please cite our [paper](https://arxiv.org/abs/2603.16273).

```bibtex
@article{lee2026genzlio,
  title={{GenZ-LIO: Generalizable LiDAR-Inertial Odometry Beyond Confined--Open Boundaries}},
  author={Lee, Daehan and Lim, Hyungtae and Kim, Seongjun and Rho, Soonbin and Lee, Changhyeon and Park, Sanghyun and Hong, Junwoo and Choi, Eunseon and Jo, Hyunyoung and Han, Soohee},
  journal={arXiv preprint arXiv:2603.16273},
  year={2026}
}
```

For LiDAR-only odometry, see [GenZ-ICP](https://github.com/cocel-postech/genz-icp)
([arXiv](https://arxiv.org/abs/2411.06766), [IEEE *Xplore*](https://ieeexplore.ieee.org/document/10753079)).

```bibtex
@article{lee2024genzicp,
  author={Lee, Daehan and Lim, Hyungtae and Han, Soohee},
  title={{GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting}},
  journal={IEEE Robotics and Automation Letters (RA-L)},
  year={2025},
  volume={10},
  number={1},
  pages={152--159},
  doi={10.1109/LRA.2024.3498779}
}
```

## :sparkles: Contributors

Bug reports, documentation improvements, and pull requests are always welcome.

<a href="https://github.com/cocel-postech/genz-lio/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=cocel-postech/genz-lio" />
</a>

## :pray: Acknowledgments and license

Many thanks to the [HKU-MARS Lab](https://github.com/hku-mars) and [PRBonn](https://github.com/PRBonn) for their open-source contributions to the robotics community.

GenZ-LIO builds on [PV-LIO](https://github.com/HViktorTsoi/PV-LIO),
[VoxelMap](https://github.com/hku-mars/VoxelMap),
[FAST-LIO](https://github.com/hku-mars/FAST_LIO), and
[IKFoM](https://github.com/hku-mars/IKFoM). We also thank the
[KISS-ICP](https://github.com/PRBonn/kiss-icp) project.

GenZ-LIO is distributed under [GPL-2.0](https://github.com/cocel-postech/genz-lio/blob/master/LICENSE); dependency modifications are listed in [PATCHES.md](https://github.com/cocel-postech/genz-lio/blob/master/cpp/genz_lio/3rdparty/PATCHES.md).

## :mailbox: Contact

For questions and bugs, open an
[issue](https://github.com/cocel-postech/genz-lio/issues) or contact
[Daehan Lee](https://github.com/Daehan2Lee) ( :envelope: daehanlee `at` postech `dot` ac `dot` kr)
