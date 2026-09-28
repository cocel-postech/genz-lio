#!/usr/bin/env bash
# Build the standalone offline bag tool, then forward its arguments.
set -eo pipefail
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != noetic ]]; then
    echo "Use a fresh ROS 1 Noetic shell for bag preparation." >&2
    exit 1
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "ROS 1 Noetic is required for this preparation tool; see python/README.md." >&2
    exit 1
fi
tool_args=("$@")
set --
source /opt/ros/noetic/setup.bash
set -u
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${GENZ_BAG_TOOLS_BUILD_DIR:-${script_dir}/../../build/bag_tools}"
cmake -S "$script_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 >&2
cmake --build "$build_dir" --parallel 2 >&2
exec "$build_dir/prepare_rosbag" "${tool_args[@]}"
