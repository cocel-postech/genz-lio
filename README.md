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

<img src="pictures/GenZ-ICP.gif" width="500" alt="Temporary illustration from GenZ-ICP" />
<br />
<br />

</div>

## About GenZ-LIO

[GenZ-LIO](https://arxiv.org/abs/2603.16273) estimates LiDAR-inertial odometry
across confined spaces, open environments, and transitions between them.
It was evaluated on 42 sequences
from nine public datasets and our [NarrowWide dataset](https://github.com/cocel-postech/NarrowWide) and supports **Python, ROS 1, and ROS 2**.

<details>
<summary>Algorithm overview</summary>

- **Scale-aware adaptive voxelization** adjusts scan downsampling to the environment.
- **Hybrid-metric state update** combines point-to-plane and point-to-point constraints.
- **Voxel-pruned correspondence search** reduces the cost of point-to-point matching.

</details>

## Python support

Process recorded data with an optional visualizer; no ROS installation is needed.

<details>
<summary>Install, build, and run (Linux, Python 3.8–3.12)</summary>

**1. Install dependencies and get the source**

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

**2. Build and install**

Pip builds the C++ extension and installs rosbag and visualization dependencies:

```bash
python -m pip install '.[rosbag,viz]'
```

**3. Prepare configurations**

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

**4. Run**

**Benchmark example: NarrowWide Handheld-A-01**

Use the experiment YAML prepared above with the
[NarrowWide](https://github.com/cocel-postech/NarrowWide) Handheld-A-01 recording:

```bash
genz_lio_pipeline run /data/Handheld-A-01.bag \
    --config configs/experiments/narrowwide/vlp16_handheld_a_01.yaml \
    --visualize --image-topic /camera/color/image_raw/compressed \
    --output results/narrowwide_handheld_a_01
```

Replace `/data/Handheld-A-01.bag` with your bag path. The experiment YAML already
contains the benchmark sensor topics and calibration. Obtain the dataset separately.
The optional `--image-topic` displays the camera images recorded in this bag;
omit it to run without the camera panel. See [camera preview](python/README.md#camera-preview).

Press **SPACE** to start. For rosbag2, pass the recording directory instead of a
`.bag` file. The exported YAMLs also work outside the source checkout.

For your own sensor, copy a template, calibrate it, then open the visualizer:

```bash
cp configs/default/velodyne.yaml my_robot.yaml
# Edit my_robot.yaml for your sensor topics and LiDAR-to-IMU calibration.
genz_lio_pipeline run /data/sequence.bag \
    --config my_robot.yaml --visualize --output results/my_run
```

To show camera images from your own recording, add `--image-topic` followed by
an Image or CompressedImage topic present in that bag.

After the PyPI release, `python -m pip install 'genz-lio[rosbag,viz]'` will replace
the source build above. A compatible wheel needs no local C++ build; plain
`pip install genz-lio` installs only the core and CLI.

</details>

See [python/README.md](python/README.md) for installation options, input formats,
configuration export, visualization controls, and saved trajectories.

## ROS 1 support

<details>
<summary>Install, build, and run with RViz (Noetic)</summary>

**1. Install dependencies**

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

**Livox CustomMsg input:** first [build and source livox_ros_driver](ros/README.md#livox-ros-1).
Complete that step in this shell before building GenZ-LIO below. Skip it for
PointCloud2 input.

**2. Build GenZ-LIO**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/cocel-postech/genz-lio.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

**3. Run**

Choose a calibrated sensor or experiment YAML. The launch file starts RViz:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch genz_lio odometry.launch config:=default/velodyne.yaml
```

In a second terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rosbag play /data/sequence.bag
```

For live input, start your LiDAR/IMU drivers instead of the bag player.

</details>

See [ros/README.md](ros/README.md#ros-1-noetic) for Livox setup, configuration,
recording outputs, and troubleshooting.

## ROS 2 support

<details>
<summary>Install, build, and run with RViz (Humble / Jazzy)</summary>

**1. Install dependencies**

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

**Livox CustomMsg input:** first [build and source livox_ros_driver2](ros/README.md#livox-ros-2).
Complete that step in this shell before building GenZ-LIO below. Skip it for
PointCloud2 input.

**2. Build GenZ-LIO**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/cocel-postech/genz-lio.git
cd ~/ros2_ws
colcon build --packages-select genz_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**3. Run**

Choose a calibrated sensor or experiment YAML. The launch file starts RViz:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch genz_lio odometry.launch.py config:=default/velodyne.yaml
```

In a second terminal, source the same workspace before playing a rosbag2 directory:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play /data/sequence_ros2
```

For live input, start your LiDAR/IMU drivers instead of the bag player.

</details>

See [ros/README.md](ros/README.md#ros-2-humble--jazzy) for Livox setup, DDS,
[benchmark configurations](ros/README.md#benchmark-configurations), and recording outputs.

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
Contributions are greatly appreciated, and we would be happy to see your profile appear below!

<a href="https://github.com/cocel-postech/genz-lio/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=cocel-postech/genz-lio" />
</a>

## :pray: Acknowledgments and license

Many thanks to the [HKU-MARS Lab](https://github.com/hku-mars) and the KISS-ICP team at [PRBonn](https://github.com/PRBonn) for their open-source contributions to the robotics community.

GenZ-LIO builds on [PV-LIO](https://github.com/HViktorTsoi/PV-LIO),
[VoxelMap](https://github.com/hku-mars/VoxelMap),
[FAST-LIO](https://github.com/hku-mars/FAST_LIO), and
[IKFoM](https://github.com/hku-mars/IKFoM). We also thank the
[KISS-ICP](https://github.com/PRBonn/kiss-icp) project.

GenZ-LIO is distributed under [GPL-2.0](https://github.com/cocel-postech/genz-lio/blob/master/LICENSE); dependency modifications are listed in [PATCHES.md](https://github.com/cocel-postech/genz-lio/blob/master/cpp/genz_lio/3rdparty/PATCHES.md).

## :mailbox: Contact

For questions and bug reports, open an
[issue](https://github.com/cocel-postech/genz-lio/issues) or contact
[Daehan Lee](https://github.com/Daehan2Lee) ( :envelope: daehanlee `at` postech `dot` ac `dot` kr)
