// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Mapping points and their uncertainty from the LiDAR frame into the world
// frame, given the current filter state.
#pragma once

#include "StateIkfom.hpp"
#include "Types.hpp"

namespace genz_lio {

/// Point in the world frame: LiDAR -> IMU body via the extrinsic, then body ->
/// world via the estimated pose.
inline Vec3 transformPointToWorld(const state_ikfom &state, const Vec3 &point_lidar) {
    return state.rot * (state.offset_R_L_I * point_lidar + state.offset_T_L_I) + state.pos;
}

/// Propagates a point's measurement covariance into the world frame, carrying
/// through the uncertainty of both the extrinsic and the pose.
Mat3 transformCovarianceToWorld(const Vec3 &point_lidar, const EsekfState &kf,
                                const Mat3 &cov_lidar);

}  // namespace genz_lio
