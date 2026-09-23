// This file is part of GenZ-LIO, released under the GNU GPL v2.
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <vector>

namespace genz_lio {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec3i = Eigen::Vector3i;
using Vec3f = Eigen::Vector3f;
using Mat3f = Eigen::Matrix3f;

/// One LiDAR return, in whichever frame the containing cloud is expressed in.
struct Point {
    Vec3 position = Vec3::Zero();
    float intensity = 0.0f;
    /// Offset from the start of the scan, in seconds.
    float time_offset = 0.0f;
};

using PointCloud = std::vector<Point>;

/// A single IMU sample.
struct ImuSample {
    double timestamp = 0.0;
    Vec3 linear_acceleration = Vec3::Zero();
    Vec3 angular_velocity = Vec3::Zero();
};

/// A scan paired with the IMU samples spanning it.
struct MeasurementBundle {
    double scan_begin_time = 0.0;
    double scan_end_time = 0.0;
    PointCloud scan;
    std::vector<ImuSample> imu;
};

/// A rigid transform, stored as rotation matrix and translation.
struct Pose {
    Mat3 rotation = Mat3::Identity();
    Vec3 translation = Vec3::Zero();

    Vec3 operator*(const Vec3 &p) const { return rotation * p + translation; }
};

/// A map point carrying the measurement uncertainty derived from the LiDAR
/// ranging model. `cov_lidar` is the covariance in the sensor frame; `cov` is
/// the same uncertainty propagated into the world frame.
struct PointWithCov {
    Vec3 point = Vec3::Zero();
    Vec3 point_world = Vec3::Zero();
    Mat3 cov = Mat3::Zero();
    Mat3 cov_lidar = Mat3::Zero();
};

/// A locally fitted plane and the uncertainty of its parameters.
struct Plane {
    Vec3 center = Vec3::Zero();
    Vec3 normal = Vec3::Zero();
    Vec3 x_normal = Vec3::Zero();
    Vec3 y_normal = Vec3::Zero();
    Mat3 covariance = Mat3::Zero();
    Mat6 plane_cov = Mat6::Zero();

    float radius = 0.0f;
    float min_eigen_value = 1.0f;
    float mid_eigen_value = 1.0f;
    float max_eigen_value = 1.0f;
    float d = 0.0f;

    int points_size = 0;
    int num_out_of_plane = 0;
    int id = 0;

    bool is_plane = false;
    bool is_init = false;
    bool update_enable = true;

    // Only used when publishing the map for visualization.
    bool is_update = false;
    int last_update_points_size = 0;
};

/// A point-to-plane correspondence.
struct PointToPlaneMatch {
    Vec3 point = Vec3::Zero();
    Vec3 point_world = Vec3::Zero();
    Vec3 normal = Vec3::Zero();
    Vec3 center = Vec3::Zero();
    Mat6 plane_cov = Mat6::Zero();
    Mat3 cov_lidar = Mat3::Zero();
    double d = 0.0;
    int layer = 0;
};

/// A point-to-point correspondence. `num_neighbors` and `num_candidate_voxels`
/// record how much of the map the pruned search had to touch, and feed the
/// discretization variance of the combined covariance.
struct PointToPointMatch {
    Vec3 point = Vec3::Zero();
    Vec3 point_world = Vec3::Zero();
    Vec3 point_center = Vec3::Zero();
    Mat3 point_cov = Mat3::Zero();
    Mat3 cov_lidar = Mat3::Zero();
    std::size_t num_neighbors = 0;
    std::size_t num_candidate_voxels = 0;
};

/// Integer coordinates of a voxel. Constructed from a position already divided
/// by the voxel size.
class VoxelKey {
public:
    std::int64_t x = 0, y = 0, z = 0;

    VoxelKey() = default;

    explicit VoxelKey(const Vec3 &scaled_point)
        : x(static_cast<std::int64_t>(std::floor(scaled_point.x()))),
          y(static_cast<std::int64_t>(std::floor(scaled_point.y()))),
          z(static_cast<std::int64_t>(std::floor(scaled_point.z()))) {}

    VoxelKey(std::int64_t x_, std::int64_t y_, std::int64_t z_) : x(x_), y(y_), z(z_) {}

    bool operator==(const VoxelKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash {
    std::size_t operator()(const VoxelKey &voxel) const {
        // Mix all 64 bits of all three coordinates. Reinterpreting this int64
        // struct as three uint32 values omitted z and violated strict aliasing.
        const auto mix = [](std::uint64_t v) {
            v += 0x9e3779b97f4a7c15ULL;
            v = (v ^ (v >> 30)) * 0xbf58476d1ce4e5b9ULL;
            v = (v ^ (v >> 27)) * 0x94d049bb133111ebULL;
            return v ^ (v >> 31);
        };
        const auto y = mix(static_cast<std::uint64_t>(voxel.y));
        const auto z = mix(static_cast<std::uint64_t>(voxel.z));
        const auto h = mix(static_cast<std::uint64_t>(voxel.x)) ^
                       ((y << 21) | (y >> 43)) ^ ((z << 42) | (z >> 22));
        return static_cast<std::size_t>(sizeof(std::size_t) < 8 ? h ^ (h >> 32) : h);
    }
};

}  // namespace genz_lio
