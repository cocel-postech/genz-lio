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
[Install](#installation)
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
It supports **Python, ROS 1, and ROS 2**, and was evaluated on 42 sequences
from nine public datasets and our [NarrowWide dataset](https://github.com/cocel-postech/NarrowWide).

<details>
<summary>Algorithm overview</summary>

- **Scale-aware adaptive voxelization** adjusts scan downsampling to the environment.
- **Hybrid-metric state update** combines point-to-plane and point-to-point constraints.
- **Voxel-pruned correspondence search** reduces the cost of point-to-point matching.

</details>

## Installation

### Python

Follow the [Python installation guide](python/README.md#installation) for dependencies
and virtual environment setup. From the repository root:

```bash
python -m pip install '.[rosbag,viz]'
genz_lio_pipeline --help
```

See [python/README.md](python/README.md) for input formats, visualization,
and saving trajectories.

## ROS support

Build instructions, dependencies, and dataset playback are in
[ros/README.md](ros/README.md#installation).

<details>
<summary>ROS 1 Noetic</summary>

After building and sourcing the catkin workspace:

```bash
roslaunch genz_lio odometry.launch config:=default/velodyne.yaml
```

</details>

<details>
<summary>ROS 2 Humble / Jazzy</summary>

After building and sourcing the colcon workspace:

```bash
ros2 launch genz_lio odometry.launch.py config:=default/velodyne.yaml
```

</details>

Both launch RViz by default. Select a calibrated sensor or experiment YAML.
See [benchmark configurations](ros/README.md#benchmark-configurations) and the
[parameter guide](ros/config/parameter_tuning_guide.md).

## Citation

If you use GenZ-LIO, please cite our [paper](https://arxiv.org/abs/2603.16273).

<details>
<summary>GenZ-LIO — BibTeX</summary>

```bibtex
@article{lee2026genzlio,
  title={{GenZ-LIO: Generalizable LiDAR-Inertial Odometry Beyond Confined--Open Boundaries}},
  author={Lee, Daehan and Lim, Hyungtae and Kim, Seongjun and Rho, Soonbin and Lee, Changhyeon and Park, Sanghyun and Hong, Junwoo and Choi, Eunseon and Jo, Hyunyoung and Han, Soohee},
  journal={arXiv preprint arXiv:2603.16273},
  year={2026}
}
```

</details>

For LiDAR-only odometry, see [GenZ-ICP](https://github.com/cocel-postech/genz-icp)
([paper](https://arxiv.org/abs/2411.06766), [IEEE Xplore](https://ieeexplore.ieee.org/document/10753079)).

<details>
<summary>GenZ-ICP — BibTeX</summary>

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

</details>

## Contributors

Bug reports, documentation improvements, and pull requests are welcome.
See [all contributors](https://github.com/cocel-postech/genz-lio/graphs/contributors).

<a href="https://github.com/Daehan2Lee">
  <img src="https://github.com/Daehan2Lee.png?size=80" width="64" height="64" alt="Daehan Lee" />
</a>

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
