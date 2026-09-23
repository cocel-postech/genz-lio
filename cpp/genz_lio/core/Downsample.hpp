// This file is part of GenZ-LIO, released under the GNU GPL v2.
#pragma once

#include "Types.hpp"

namespace genz_lio {

/// Uniform voxel grid downsampling: every occupied cell of edge length
/// `leaf_size` contributes the centroid of the points that fell into it,
/// averaging position, intensity and time offset alike.
///
/// A non-positive `leaf_size` returns the input unchanged.
PointCloud voxelDownsample(const PointCloud &cloud, double leaf_size);

/// Sorts a scan by per-point time offset, ascending.
void sortByTime(PointCloud &cloud);

}  // namespace genz_lio
