// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Renders the fitted planes of the voxel map as RViz markers, coloured by how
// uncertain each plane is.
#pragma once

#include "RosCompat.hpp"
#include "core/VoxelMap.hpp"

#include <string>

namespace genz_lio {
namespace ros_wrapper {

/// Renders every fitted plane down to `max_layer` as a flat cylinder, sized by
/// the plane's eigenvalues and coloured from blue (certain) to red (uncertain).
/// Costly on a large map — meant for inspection, not for continuous use.
MarkerArray buildVoxelMapMarkers(const VoxelMapType &voxel_map, int max_layer,
                                 const std::string &frame, const Time &stamp);

}  // namespace ros_wrapper
}  // namespace genz_lio
