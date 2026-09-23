<div align="center">

# GenZ-LIO

**Generalizable LiDAR-Inertial Odometry Beyond Confined–Open Boundaries**

[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](cpp/genz_lio)
[![Python](https://img.shields.io/badge/Python-3.8--3.12-blue)](python/README.md)
[![ROS 1](https://img.shields.io/badge/ROS%201-Noetic-blue)](ros/README.md)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)](ros/README.md)
[![License](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](LICENSE)

[Paper](https://arxiv.org/abs/2603.16273) ·
[Install](#installation) ·
[Python](python/README.md) ·
[ROS / ROS 2](ros/README.md) ·
[Parameter guide](ros/config/parameter_tuning_guide.md) ·
[NarrowWide](https://github.com/cocel-postech/NarrowWide)

<img src="pictures/GenZ-ICP.gif" width="600" alt="Temporary illustration from GenZ-ICP" />

*Temporary illustration from [GenZ-ICP](https://github.com/cocel-postech/genz-icp),
to be replaced with a GenZ-LIO demonstration before release.*

</div>

GenZ-LIO estimates motion from LiDAR and IMU measurements across confined spaces,
open terrain, and transitions between them. A shared C++ estimator provides
ROS 1, ROS 2, and Python interfaces.

- **Scale-aware adaptive voxelization** adjusts scan downsampling as the spatial scale changes.
- **Hybrid-metric state update** combines point-to-plane and point-to-point constraints.
- **Voxel-pruned correspondence search** reduces the search work for point-to-point matching.

The [paper](https://arxiv.org/abs/2603.16273) evaluates 42 sequences from nine
public datasets and our [NarrowWide dataset](https://github.com/cocel-postech/NarrowWide).
Sequence-specific configurations and their mapping are listed in the
[ROS guide](ros/README.md#benchmark-configurations). Sensor defaults are starting
points for a new platform; use the experiment configurations for benchmark runs.

## Installation

### Python

Install from a complete source checkout on Linux. The Python pipeline can read
rosbags without a ROS installation. Building its C++ extension requires a C++17
compiler, CMake 3.16+, OpenMP, Eigen 3.4+, and Boost headers. The interactive
visualizer uses Polyscope and requires an OpenGL-capable display.

```bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libeigen3-dev libboost-dev \
    python3-dev python3-venv libgl1
git clone https://github.com/cocel-postech/genz-lio.git
cd genz-lio
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install '.[rosbag,viz]'
```

If the system Eigen is older than 3.4, CMake downloads Eigen 3.4 during the build;
this requires network access. Source installation is the documented distribution
method; a PyPI release is not required by these instructions.

Inspect an input bag, then run with a calibrated configuration:

```bash
genz_lio_pipeline inspect /data/sequence.bag
genz_lio_pipeline run /data/sequence.bag \
    --config ros/config/default/velodyne.yaml --visualize --output results/my_run
```

Press **SPACE** to start. A TUM trajectory is saved when processing finishes or
you quit the visualizer normally. See the [Python guide](python/README.md) for
camera previews, output formats, controls, and the Python API.

### ROS 1 / ROS 2

Follow the [ROS build instructions](ros/README.md#installation) to install
dependencies and build with catkin or colcon. Each interface has one launch file:

```bash
# ROS 1: source the catkin workspace first.
roslaunch genz_lio odometry.launch config:=default/velodyne.yaml

# ROS 2: source the colcon workspace first, in a separate ROS 2 shell.
ros2 launch genz_lio odometry.launch.py config:=default/velodyne.yaml
```

Both open RViz by default. Start the sensor drivers or bag player separately.
Select one complete YAML using `config`; set LiDAR/IMU topics, timing, noise, and
extrinsics for your platform. Livox `CustomMsg` input additionally requires the
corresponding Livox ROS driver at build time. See the
[ROS guide](ros/README.md) for playback, DDS setup, and recording outputs.

### Docker build check

From the repository root, with Docker installed:

```bash
bash docker/build_ros2.sh humble
# Alternative build target:
bash docker/build_ros2.sh jazzy
```

The script installs dependencies in a disposable ROS container, builds the
package, runs its tests, and checks that the node starts. It downloads packages
and requires network access. This checks the build and an idle node; RViz,
GPU access, and dataset replay are not part of this script.

## Configuration and reproducibility

The same complete YAML can be used by all three interfaces. ROS resolves a
relative `config` path under the package's `config/` directory; Python resolves
`--config` relative to the current working directory. No sensor overlay is
needed. See the [parameter guide](ros/config/parameter_tuning_guide.md).

`runtime.max_threads: 0` selects the detected physical-core count without CPU
pinning. Accuracy and throughput should be checked on the intended machine;
different compilers, SIMD targets, or Eigen versions can change floating-point
results. Keep the input data, YAML, build revision, and evaluation convention
with any reported score.

## Citation

If you use GenZ-LIO, please cite our [paper](https://arxiv.org/abs/2603.16273):

```bibtex
@article{lee2026genzlio,
  title={{GenZ-LIO: Generalizable LiDAR-Inertial Odometry Beyond Confined--Open Boundaries}},
  author={Lee, Daehan and Lim, Hyungtae and Kim, Seongjun and Rho, Soonbin and Lee, Changhyeon and Park, Sanghyun and Hong, Junwoo and Choi, Eunseon and Jo, Hyunyoung and Han, Soohee},
  journal={arXiv preprint arXiv:2603.16273},
  year={2026}
}
```

## Acknowledgments and license

GenZ-LIO builds on [PV-LIO](https://github.com/HViktorTsoi/PV-LIO),
[VoxelMap](https://github.com/hku-mars/VoxelMap),
[FAST-LIO](https://github.com/hku-mars/FAST_LIO), and
[IKFoM](https://github.com/hku-mars/IKFoM). We also thank the
[KISS-ICP](https://github.com/PRBonn/kiss-icp) and
[GenZ-ICP](https://github.com/cocel-postech/genz-icp) projects.

GenZ-LIO is distributed under [GPL-2.0](LICENSE). Notices in vendored code are
preserved; local dependency modifications are documented in
[PATCHES.md](cpp/genz_lio/3rdparty/PATCHES.md). The temporary GenZ-ICP illustration
retains its [upstream license](pictures/GenZ-ICP.LICENSE).

## Contact

For questions and bug reports, open an
[issue](https://github.com/cocel-postech/genz-lio/issues) or contact
[Daehan Lee](https://github.com/Daehan2Lee) at daehanlee@postech.ac.kr.
