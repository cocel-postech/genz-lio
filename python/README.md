# GenZ-LIO Python

Process recorded data with an optional visualizer; no ROS installation is needed.

## 1. Install dependencies and get the source

Until the PyPI release is published, install from this repository:

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

## 2. Build and install

Pip builds the C++ extension and installs rosbag and visualization dependencies:

```bash
python -m pip install '.[rosbag,viz]'
```

After the PyPI release, `python -m pip install 'genz-lio[rosbag,viz]'` will replace
the source build above. A compatible wheel needs no local C++ build; plain
`pip install genz-lio` installs only the core and CLI.

## 3. Prepare configurations

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

## 4. Run

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

See the [parameter guide](https://github.com/cocel-postech/genz-lio/blob/master/ros/config/parameter_tuning_guide.md) for parameter tuning.

## Additional options

<details>
<summary>Development: rebuild after C++ changes</summary>

An editable install tracks Python source changes, but C++ changes require
rebuilding the extension. A catkin/colcon build does not update it.
From the repository root:

```bash
python -m pip install -e '.[rosbag,viz]'
python -m pip install pytest
python -m pytest python/tests -q
```

</details>

### Installation options

Source builds target Linux x86-64 and Python 3.8–3.12, and require a C++17
compiler, CMake 3.16+, OpenMP, Eigen 3.4+, and Boost headers. CMake downloads
Eigen 3.4 if the system version is older. Visualization needs an active display
with OpenGL support. Run source installation commands from the repository root;
a catkin/colcon build is unnecessary for Python.

Use `python -m pip install '.[all]'` for all readers and visualization.
Individual extras are `.[rosbag]`, `.[viz]`, and `.[ouster]`; `python -m pip install .`
installs only the core and CLI. After the PyPI release, the equivalent full
installation will be `python -m pip install 'genz-lio[all]'`.

### Configurations without a source checkout

The package includes the YAMLs from `ros/config/`. Export them as shown in
step 3, or export one file:

```bash
genz_lio_pipeline list-configs
genz_lio_pipeline export-config default/velodyne.yaml another_robot.yaml
```

Edit the exported file and pass its path to `--config`. With a source checkout,
paths such as `--config ros/config/default/velodyne.yaml` work directly from the
repository root. Omitting `--config` uses core defaults, not a sensor YAML.

### Input options

Inspect the recording to find its topics:

```bash
genz_lio_pipeline inspect /data/sequence.bag
```

Topics default to `common.lidar_topic` and `common.imu_topic` in the YAML.
Override them when needed:

```bash
genz_lio_pipeline run /data/sequence.bag \
    --config my_robot.yaml \
    --lidar-topic /velodyne_points --imu-topic /imu/data \
    --output results/my_run
```

Omit `--visualize` for headless processing. Use `--visualize-autoplay` to start
visualization immediately and close it at completion. Processing runs without
a real-time pacing limit; sensor timestamps determine estimation timing.

### Camera input

`--image-topic` requires visualization and an image topic inside the input bag:
`sensor_msgs/Image` or `sensor_msgs/CompressedImage`. It does not subscribe to a
live ROS topic. Images are display-only; omitting the option skips image reading
and hides the camera panel.

### Save odometry

Trajectories are saved with or without visualization. `--output` selects the
output directory (default: `results`). Use `--format` to select one or both formats:

```bash
genz_lio_pipeline run /data/sequence.bag \
    --config my_robot.yaml --visualize \
    --output results/my_run --format tum --format kitti
```

| Format | File | Row contents |
|---|---|---|
| TUM (default) | `<sequence>_tum.txt` | `timestamp tx ty tz qx qy qz qw` |
| KITTI | `<sequence>_kitti.txt` | First three rows of the 4×4 pose, flattened to 12 values |

Poses map IMU/body coordinates into the estimator world frame, with translation
in meters and TUM timestamps in seconds. Files are written at completion or
normal quit (**Q** or closing the window), including the processed portion of
an interrupted run. Forcibly killing the process can prevent saving. Reusing
the same output directory and sequence name overwrites those trajectory files.

<details>
<summary>Ouster PCAP input</summary>

Install the `ouster` or `all` extra. For a single-sensor PCAP, place the matching
metadata JSON beside the capture with the same filename stem, then pass the
`.pcap` path to `genz_lio_pipeline run`. Native IMU packets must be present.

The reader converts acceleration from g to m/s² and angular velocity from
degrees/s to rad/s using sensor timestamps. Configure LiDAR/IMU extrinsics for
the SDK's sensor coordinates, and split timestamp resets into separate recordings.
The Ouster SDK extra requires glibc 2.28 or newer; the bag reader does not need it.

</details>

<details>
<summary>Python API</summary>

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

</details>
