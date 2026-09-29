#!/usr/bin/env bash
# Build the ROS 2 wrapper inside a container, so the result does not depend on
# what happens to be installed on the host.
#
#   docker/build_ros2.sh humble
#   docker/build_ros2.sh jazzy
#
# The container isolates build dependencies from the host installation.
# Dependency downloads and Docker access are required.
set -euo pipefail

DISTRO="${1:-humble}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker run --rm -t \
  -v "$REPO:/ws/src/genz-lio:ro" \
  -w /ws \
  "ros:${DISTRO}-ros-base" \
  bash -lc '
# ROS setup scripts reference unset variables, so no -u here.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
# Colcon otherwise starts one compiler per CPU, which can exhaust memory.
export MAKEFLAGS=-j2
apt-get update -qq
apt-get install -y -qq --no-install-recommends \
    build-essential cmake \
    libeigen3-dev libboost-dev libyaml-cpp-dev libpcl-dev \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-visualization-msgs \
    python3-colcon-common-extensions >/dev/null

source /opt/ros/$ROS_DISTRO/setup.bash
echo "=== building for ROS 2 $ROS_DISTRO ==="
colcon build --packages-select genz_lio \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+ 2>&1 | tail -40

echo "=== result ==="
ls -la install/genz_lio/lib/genz_lio/
source install/setup.bash
ctest --test-dir build/genz_lio --output-on-failure

# An idle node should stay alive until timeout. Do not mask a startup failure.
set +e
timeout --signal=INT --kill-after=5 10 ros2 run genz_lio genz_lio_node \
    --ros-args -p config_path:=/ws/src/genz-lio/ros/config/default/velodyne.yaml >/tmp/genz-startup.log 2>&1
startup_status=$?
set -e
cat /tmp/genz-startup.log
test "$startup_status" -eq 124
'
