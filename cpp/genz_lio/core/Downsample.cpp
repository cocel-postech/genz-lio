// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "Downsample.hpp"

#include <tsl/robin_map.h>

#include <algorithm>

namespace genz_lio {
namespace {

struct Accumulator {
    Vec3 position = Vec3::Zero();
    double intensity = 0.0;
    double time_offset = 0.0;
    int count = 0;
};

}  // namespace

PointCloud voxelDownsample(const PointCloud &cloud, const double leaf_size) {
    if (leaf_size <= 0.0 || cloud.empty()) return cloud;

    const double inv_leaf_size = 1.0 / leaf_size;

    tsl::robin_map<VoxelKey, Accumulator, VoxelHash> grid;
    grid.reserve(cloud.size());
    for (const auto &point : cloud) {
        const VoxelKey key(point.position * inv_leaf_size);
        auto &cell = grid[key];
        cell.position += point.position;
        cell.intensity += point.intensity;
        cell.time_offset += point.time_offset;
        ++cell.count;
    }

    PointCloud downsampled;
    downsampled.reserve(grid.size());
    for (const auto &entry : grid) {
        const Accumulator &cell = entry.second;
        const double inv_count = 1.0 / static_cast<double>(cell.count);
        Point centroid;
        centroid.position = cell.position * inv_count;
        centroid.intensity = static_cast<float>(cell.intensity * inv_count);
        centroid.time_offset = static_cast<float>(cell.time_offset * inv_count);
        downsampled.push_back(centroid);
    }
    return downsampled;
}

void sortByTime(PointCloud &cloud) {
    std::sort(cloud.begin(), cloud.end(),
              [](const Point &a, const Point &b) { return a.time_offset < b.time_offset; });
}

}  // namespace genz_lio
