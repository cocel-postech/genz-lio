// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "VoxelMapVisualizer.hpp"

#include <cmath>
#include <vector>

namespace genz_lio {
namespace ros_wrapper {
namespace {

/// Trace of the plane covariance at which the colour saturates.
constexpr double kMaxTrace = 0.25;
/// Compresses the colour ramp so that small differences stay visible.
constexpr double kColorGamma = 0.2;
constexpr float kPlaneAlpha = 0.8f;

/// Blue through green to red, the usual "jet" ramp.
void jetColor(double v, const double vmin, const double vmax, std::uint8_t &r, std::uint8_t &g,
              std::uint8_t &b) {
    r = g = b = 255;
    v = std::max(vmin, std::min(v, vmax));
    const double dv = vmax - vmin;

    if (v < (vmin + 0.25 * dv)) {
        r = 0;
        g = static_cast<std::uint8_t>(255 * (4 * (v - vmin) / dv));
    } else if (v < (vmin + 0.5 * dv)) {
        r = 0;
        b = static_cast<std::uint8_t>(255 * (1 + 4 * (vmin + 0.25 * dv - v) / dv));
    } else if (v < (vmin + 0.75 * dv)) {
        r = static_cast<std::uint8_t>(255 * (4 * (v - vmin - 0.5 * dv) / dv));
        b = 0;
    } else {
        g = static_cast<std::uint8_t>(255 * (1 + 4 * (vmin + 0.75 * dv - v) / dv));
        b = 0;
    }
}

/// Walks an octree, collecting the planes that have changed since the last publish.
void collectPlanes(const OctoTree *cell, const int max_layer, std::vector<Plane> &planes) {
    if (cell == nullptr || cell->layer_ > max_layer) return;
    if (cell->plane_ptr_->is_update) planes.push_back(*cell->plane_ptr_);

    // A cell that fits a plane has no children worth descending into.
    if (cell->layer_ < cell->max_layer_ && !cell->plane_ptr_->is_plane) {
        for (int i = 0; i < 8; ++i) collectPlanes(cell->leaves_[i], max_layer, planes);
    }
}

/// Orientation of a marker whose local axes are the plane's own frame.
/// The marker's own orientation type, which is spelled differently in ROS 1 and
/// ROS 2 but is the same message either way.
using Quaternion = decltype(Marker().pose.orientation);

Quaternion planeOrientation(const Plane &plane) {
    Mat3 axes;
    axes << plane.x_normal(0), plane.x_normal(1), plane.x_normal(2), plane.y_normal(0),
        plane.y_normal(1), plane.y_normal(2), plane.normal(0), plane.normal(1), plane.normal(2);
    const Eigen::Quaterniond rotation(Mat3(axes.transpose()));

    Quaternion q;
    q.w = rotation.w();
    q.x = rotation.x();
    q.y = rotation.y();
    q.z = rotation.z();
    return q;
}

}  // namespace

MarkerArray buildVoxelMapMarkers(const VoxelMapType &voxel_map, const int max_layer,
                                 const std::string &frame, const Time &stamp) {
    std::vector<Plane> planes;
    for (const auto &entry : voxel_map) collectPlanes(entry.second, max_layer, planes);

    MarkerArray markers;
    markers.markers.reserve(planes.size());

    for (const auto &plane : planes) {
        const double trace =
            std::min(plane.plane_cov.block<3, 3>(0, 0).diagonal().sum(), kMaxTrace) / kMaxTrace;
        std::uint8_t r, g, b;
        jetColor(std::pow(trace, kColorGamma), 0.0, 1.0, r, g, b);

        Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = stamp;
        marker.ns = "plane";
        marker.id = plane.id;
        marker.type = Marker::CYLINDER;
        marker.action = Marker::ADD;
        marker.pose.position.x = plane.center[0];
        marker.pose.position.y = plane.center[1];
        marker.pose.position.z = plane.center[2];
        marker.pose.orientation = planeOrientation(plane);
        // Two of the eigenvalues span the disc, the third gives it thickness.
        marker.scale.x = 3 * std::sqrt(plane.max_eigen_value);
        marker.scale.y = 3 * std::sqrt(plane.mid_eigen_value);
        marker.scale.z = 2 * std::sqrt(plane.min_eigen_value);
        // Cells that never became planar are collected but drawn invisible, so
        // that their marker ids stay stable across publishes.
        marker.color.a = plane.is_plane ? kPlaneAlpha : 0.0f;
        marker.color.r = r / 256.0f;
        marker.color.g = g / 256.0f;
        marker.color.b = b / 256.0f;
        markers.markers.push_back(marker);
    }
    return markers;
}

}  // namespace ros_wrapper
}  // namespace genz_lio
