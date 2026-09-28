# GenZ-LIO for ROS 1 and ROS 2

Both interfaces use the same C++ estimator and YAML configuration format.
Follow the four steps for your ROS version below. Use a separate shell and
workspace for ROS 1 Noetic, ROS 2 Humble, and ROS 2 Jazzy.

<a id="ros-1-noetic"></a>

## ROS 1 support

### 1. Install dependencies

Start with [ROS 1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) installed:

```bash
source /opt/ros/noetic/setup.bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev libboost-dev \
    libyaml-cpp-dev ros-noetic-roscpp ros-noetic-roslib \
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
catkin_make -DCMAKE_BUILD_TYPE=Release
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
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

If using Livox CustomMsg, check that CMake reports
`Livox CustomMsg support enabled`. If the driver was added after GenZ-LIO was
built, source it and rebuild GenZ-LIO.

### 3. Prepare configurations

The package includes `config/experiments/` YAMLs for reproducing the paper's
benchmarks and `config/default/` templates for your own sensor. `config:=` accepts
a path relative to the package's `config/` directory, or an absolute YAML path.
ROS does not require the Python configuration export step.

For benchmark runs, use the matching [experiment YAML](#benchmark-configurations).
For your own sensor, copy a template; for example, with a Velodyne LiDAR:

```bash
cp ~/catkin_ws/src/genz-lio/ros/config/default/velodyne.yaml ~/my_robot.yaml
```

Edit `my_robot.yaml` for your topics, sensor type, per-point timing, noise, and
LiDAR-to-IMU calibration. The default extrinsics are placeholders.
Parameter tuning guidance is available in the [parameter guide](config/parameter_tuning_guide.md).

### 4. Run

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
rosbag play "{path_to_bag}/{NW_Handheld-A-01}.bag"
```

Replace the bag placeholders with the actual downloaded or prepared file.
Dataset downloads and input preparation notes are listed in the
[benchmark sequence guide](../python/README.md#4-run).
GenZ-LIO consumes PointCloud2 or supported Livox CustomMsg plus IMU messages;
raw Velodyne packets must first be decoded to PointCloud2 with point timing.

**2) For your own sensor**

Launch with the calibrated file from step 3:

```bash
roslaunch genz_lio odometry.launch config:="$HOME/my_robot.yaml"
```

In the second terminal, play your own recording instead of the benchmark above.
For live input, start your LiDAR/IMU drivers instead of the bag player.
The ROS node publishes `/Odometry`; it does not automatically save a trajectory.

<a id="ros-2-humble--jazzy"></a>

## ROS 2 support

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
    ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-pcl-conversions \
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
bash build.sh "$ROS_DISTRO"
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
colcon build --packages-select genz_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If using Livox CustomMsg, check that CMake reports
`Livox CustomMsg support enabled`. If the driver was added after GenZ-LIO was
built, source it and rebuild GenZ-LIO.

### 3. Prepare configurations

The package includes `config/experiments/` YAMLs for reproducing the paper's
benchmarks and `config/default/` templates for your own sensor. `config:=` accepts
a path relative to the package's `config/` directory, or an absolute YAML path.
ROS does not require the Python configuration export step.

For benchmark runs, use the matching [experiment YAML](#benchmark-configurations).
For your own sensor, copy a template; for example, with a Velodyne LiDAR:

```bash
cp ~/ros2_ws/src/genz-lio/ros/config/default/velodyne.yaml ~/my_robot.yaml
```

Edit `my_robot.yaml` for your topics, sensor type, per-point timing, noise, and
LiDAR-to-IMU calibration. The default extrinsics are placeholders.
Parameter tuning guidance is available in the [parameter guide](config/parameter_tuning_guide.md).

### 4. Run

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
ros2 bag play "{path_to_rosbag2_recording}"
```

Use a rosbag2 recording of Handheld-A-01 with the original sensor topics,
timestamps, and message fields. The command above expects a rosbag2 directory,
not a ROS 1 `.bag`; prepare the recording in ROS 2 format first if needed.
Source `install/setup.bash` in every terminal, including the bag player and
drivers. With Fast DDS, this also applies the package's default transport profile
unless you have already selected your own profile.
Dataset downloads and input preparation notes are listed in the
[benchmark sequence guide](../python/README.md#4-run).
GenZ-LIO consumes PointCloud2 or supported Livox CustomMsg plus IMU messages;
raw Velodyne packets must first be decoded to PointCloud2 with point timing.

**2) For your own sensor**

Launch with the calibrated file from step 3:

```bash
ros2 launch genz_lio odometry.launch.py config:="$HOME/my_robot.yaml"
```

In the second terminal, play your own recording instead of the benchmark above.
For live input, start your LiDAR/IMU drivers instead of the bag player.
The ROS node publishes `/Odometry`; it does not automatically save a trajectory.

<a id="additional-options"></a>

## + Additional options

### Development: rebuild after C++ changes

Rebuild the ROS workspace after changing the C++ core or ROS wrapper, then
source its setup file again. Stop and restart the running node to load the new
binary. These commands do not rebuild the Python extension.

```bash
# ROS 1, in a Noetic shell:
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

```bash
# ROS 2, in the matching Humble or Jazzy shell:
cd ~/ros2_ws
colcon build --packages-select genz_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For newly added Livox CustomMsg support, source the driver workspace before
rebuilding. Set `lidar_type: livox` for CustomMsg or `lidar_type: livox_pcl`
for Livox PointCloud2. The message type must match the selected ROS wrapper.

---

### Save odometry and point clouds

Start the recorder before playing the input, and stop it normally after processing.

The ROS node does not automatically write a TUM trajectory. Record its output:

```bash
# ROS 1, in a separate sourced terminal:
rosbag record -O genz_output.bag /Odometry /tf /tf_static
# ROS 2, in a separate sourced terminal:
ros2 bag record -o genz_output /Odometry /tf /tf_static
```

These commands save ROS messages. For direct TUM/KITTI text output, use the
[Python pipeline](../python/README.md#save-odometry). `pcd_save.enable` saves
point clouds, not odometry: use a writable `pcd_save.directory` and a positive
`pcd_save.interval` to split output. Accumulating an entire run with interval
`-1` can use substantial memory.

---

### Launch and input options

| Launch argument | Default | Meaning |
|---|---|---|
| `config` | `default/velodyne.yaml` | Complete YAML, relative to package `config/` or absolute |
| `rviz` | `true` | Start RViz with the supplied display configuration |
| `lidar_topic` | empty | Override the YAML topic; empty keeps its value |
| `imu_topic` | empty | Override the YAML topic; empty keeps its value |

For example, a calibrated file outside the package:

```bash
ros2 launch genz_lio odometry.launch.py config:=/data/my_robot.yaml \
    lidar_topic:=/points imu_topic:=/imu/data rviz:=true
```

Add `rviz:=false` to either launch command to run without RViz. Use a fresh
shell when switching ROS distributions or workspaces. Available templates are
`avia.yaml`, `hesai.yaml`, `mid360.yaml`, `ouster.yaml`, `robosense.yaml`, and
`velodyne.yaml` under the package configuration directory.

---

### ROS 2 transport setup

DDS (Data Distribution Service) is the middleware layer used by ROS 2 to move
messages between processes. When Fast DDS is used, sourcing this workspace
selects the installed [fastdds_local.xml](config/dds/fastdds_local.xml) profile
unless `FASTRTPS_DEFAULT_PROFILES_FILE` or `FASTDDS_DEFAULT_PROFILES_FILE` is
already set. The launch file supplies the same fallback for its own processes.

Source `install/setup.bash` in **every** terminal used for the node, RViz, bag
player, or drivers. This gives separately launched processes the same default
profile. Check its resolved installed path with:

```bash
printenv FASTRTPS_DEFAULT_PROFILES_FILE
```

The profile adjusts local Fast DDS transport buffers. It does not change
estimator parameters, enforce a ROS middleware implementation, or apply to
Cyclone DDS. Transport buffers are separate from `common.qos_depth`, which limits
subscriber history. An incompatible reliability setting or sustained overload
still needs to be fixed. A larger queue can consume more memory, particularly
for high-resolution clouds.

---

### Benchmark configurations

Paths below are relative to `ros/config/`. Each file is complete and can also
be passed to Python with its repository-relative or absolute path. Without a
checkout, use [configuration export](../python/README.md#3-prepare-configurations). A shared row
means those sequences use the same YAML. Dataset information for
[NarrowWide is maintained here](https://github.com/cocel-postech/NarrowWide).

| Sequence(s) | Configuration |
|---|---|
| GD Stairs | [experiments/geode/vlp16_stairs.yaml](config/experiments/geode/vlp16_stairs.yaml) |
| GD Waterways-Short | [experiments/geode/vlp16_waterways_short.yaml](config/experiments/geode/vlp16_waterways_short.yaml) |
| GD Waterways-Medium | [experiments/geode/vlp16_waterways_medium.yaml](config/experiments/geode/vlp16_waterways_medium.yaml) |
| GD Waterways-Long | [experiments/geode/vlp16_waterways_long.yaml](config/experiments/geode/vlp16_waterways_long.yaml) |
| GD Offroad-02, GD Offroad-04, GD Offroad-07 | [experiments/geode/vlp16_offroad.yaml](config/experiments/geode/vlp16_offroad.yaml) |
| EW Katzensee-S, EW Katzensee-D, EW Intersection-S, EW Intersection-D | [experiments/enwide/os128_enwide.yaml](config/experiments/enwide/os128_enwide.yaml) |
| NV SPMS-01 | [experiments/ntu_viral/os16_spms_01.yaml](config/experiments/ntu_viral/os16_spms_01.yaml) |
| NV SPMS-02 | [experiments/ntu_viral/os16_spms_02.yaml](config/experiments/ntu_viral/os16_spms_02.yaml) |
| NV SPMS-03 | [experiments/ntu_viral/os16_spms_03.yaml](config/experiments/ntu_viral/os16_spms_03.yaml) |
| SL Cave-01, SL Cave-02, SL Cave-04 | [experiments/superloc/vlp16_cave.yaml](config/experiments/superloc/vlp16_cave.yaml) |
| SL Corridor-02 | [experiments/superloc/vlp16_corridor_02.yaml](config/experiments/superloc/vlp16_corridor_02.yaml) |
| NW Tracked-01 | [experiments/narrowwide/mid70_tracked_01.yaml](config/experiments/narrowwide/mid70_tracked_01.yaml) |
| NW Tracked-02 | [experiments/narrowwide/mid70_tracked_02.yaml](config/experiments/narrowwide/mid70_tracked_02.yaml) |
| NW Handheld-A-01 | [experiments/narrowwide/vlp16_handheld_a_01.yaml](config/experiments/narrowwide/vlp16_handheld_a_01.yaml) |
| NW Handheld-A-02 | [experiments/narrowwide/vlp16_handheld_a_02.yaml](config/experiments/narrowwide/vlp16_handheld_a_02.yaml) |
| NW Handheld-B-01, NW Handheld-B-02 | [experiments/narrowwide/avia_handheld_b.yaml](config/experiments/narrowwide/avia_handheld_b.yaml) |
| SM Long-Corridor | [experiments/subt_mrs/vlp16_long_corridor.yaml](config/experiments/subt_mrs/vlp16_long_corridor.yaml) |
| SM Multi-Floor | [experiments/subt_mrs/vlp16_multi_floor.yaml](config/experiments/subt_mrs/vlp16_multi_floor.yaml) |
| SM Laurel-Cavern | [experiments/subt_mrs/vlp16_laurel_cavern.yaml](config/experiments/subt_mrs/vlp16_laurel_cavern.yaml) |
| H21 Basement-04 | [experiments/hilti21/mid70_hilti21_basement04.yaml](config/experiments/hilti21/mid70_hilti21_basement04.yaml) |
| H22 Exp-10 | [experiments/hilti22/pandar32_hilti22_exp10.yaml](config/experiments/hilti22/pandar32_hilti22_exp10.yaml) |
| H22 Exp-16 | [experiments/hilti22/pandar32_hilti22_exp16.yaml](config/experiments/hilti22/pandar32_hilti22_exp16.yaml) |
| H22 Exp-18 | [experiments/hilti22/pandar32_hilti22_exp18.yaml](config/experiments/hilti22/pandar32_hilti22_exp18.yaml) |
| M3D Corridor-01 | [experiments/m3dgr/avia_corridor_01.yaml](config/experiments/m3dgr/avia_corridor_01.yaml) |
| M3D Corridor-02 | [experiments/m3dgr/avia_corridor_02.yaml](config/experiments/m3dgr/avia_corridor_02.yaml) |
| M3D GNSS-denial-01, M3D GNSS-denial-02 | [experiments/m3dgr/avia_gnss_denial.yaml](config/experiments/m3dgr/avia_gnss_denial.yaml) |
| H21 Drone-Arena | [experiments/hilti21/mid70_hilti21_drone_arena.yaml](config/experiments/hilti21/mid70_hilti21_drone_arena.yaml) |
| OS christ_church-01, OS christ_church-02, OS christ_church-05 | [experiments/oxford_spires/hesai64_christ_church.yaml](config/experiments/oxford_spires/hesai64_christ_church.yaml) |
| OS blenheim_palace-01, OS blenheim_palace-02, OS blenheim_palace-05 | [experiments/oxford_spires/hesai64_blenheim_palace.yaml](config/experiments/oxford_spires/hesai64_blenheim_palace.yaml) |

Use the evaluation convention associated with the dataset. Dense ground-truth
trajectory alignment, surveyed position-control errors, and M3DGR marker-based
endpoint errors are different metrics; they should not all be labeled ATE RMSE.
The supplied YAMLs identify benchmark inputs, not a guarantee of exact scores
on every build or machine. See the [parameter guide](config/parameter_tuning_guide.md).

---

### Published data

| Topic | Message | Contents |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | Estimated IMU/body pose and motion |
| `/path` | `nav_msgs/Path` | Accumulated trajectory when enabled |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Registered scan in the odometry frame |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | Body-frame scan when enabled |
| `/cloud_planar` | `sensor_msgs/PointCloud2` | Current point-to-plane matches |
| `/cloud_non_planar` | `sensor_msgs/PointCloud2` | Current point-to-point matches |

All supplied YAMLs set `publish.dense_publish_en: false`. The registered scan
is voxelized; the initialization scan has a separate display downsample.
Semantic topics publish empty clouds when their match set is empty, so RViz
can clear the previous scan. RViz's accumulated scan display and Python's
retained-map display represent the map differently.

The default TF tree has a static gravity alignment from `world` to
`camera_init` for the current initialized run, and a dynamic body transform.
Use the supplied RViz settings to keep cloud timestamps and frames consistent.

---

### Terminal information

To show the odometry information panel in the terminal, set
`publish.terminal_status_en: true` in the selected YAML. It refreshes each valid
processed scan; interactive terminals use ANSI clearing and redirected output
uses plain text. It is disabled by default. Reported FPS measures estimator
computation, not transport or rendering latency.

---

### Troubleshooting

- **No scans / initialization never completes:** check topic names and message types,
  matching ROS 2 QoS, overlapping LiDAR/IMU timestamps, and Livox build support.
- **Drift during rotation:** verify extrinsics and per-point timing before tuning.
- **RViz transform errors or lag:** check frames, timestamps, and receiver input
  completeness. Reduce bag playback speed to distinguish overload from a timing
  problem. Larger queues cannot fix sustained overload.
- **Growing memory:** inspect the retained estimator map and RViz scan history.
  Keep dense publication off. Changing `mapping.map_range` changes estimation
  history and must be checked for accuracy, not treated as a display-only fix.
- **Old launch files or DDS path:** rebuild and source the intended workspace in
  a fresh shell; check `rospack find genz_lio` or `ros2 pkg prefix genz_lio`.

For receiver-side input diagnostics, set a different writable CSV for each node:

```bash
GENZ_LIO_DIAGNOSTICS=/tmp/genz-input.csv \
    ros2 launch genz_lio odometry.launch.py config:=/data/my_robot.yaml
```

The same variable works with ROS 1. Stop normally to flush the trace. Compare
received LiDAR/IMU timestamps, processed scan bundles, and published odometry
against the input recording. A separate subscriber receiving all messages does
not prove the estimator did; a publish trace alone does not prove delivery.
Initial IMU initialization may produce no valid pose. Diagnostics are disabled
when the environment variable is unset. Keep whole-run timing separate from
accuracy and from per-scan estimator computation time.

---

### Optional Docker build check

Docker is not required to build or run GenZ-LIO. The
[build script](../docker/build_ros2.sh) installs dependencies in a disposable
container, builds the package, runs its tests, and checks that the node starts.
Use it to check a clean build without installing ROS dependencies on the host.

From the repository root, with Docker installed:

```bash
bash docker/build_ros2.sh humble
# Or:
bash docker/build_ros2.sh jazzy
```

The check requires network access for the image and dependency downloads.
It does not launch RViz, configure GPU access, or replay a dataset.
