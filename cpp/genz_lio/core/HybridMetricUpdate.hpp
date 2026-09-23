// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Hybrid-metric state update (paper Sec. V-E).
//
// Point-to-plane residuals constrain the state well where the map supports a
// reliable normal, which is typical of confined, densely sampled scenes. Out in
// the open, returns are sparser and normals less trustworthy, and point-to-point
// residuals carry more information. This module forms both and lets their
// covariances decide how much each contributes, so neither geometry has to be
// assumed in advance.
#pragma once

#include "Config.hpp"
#include "StateIkfom.hpp"
#include "Types.hpp"
#include "VoxelMap.hpp"

#include <vector>

namespace genz_lio {

class HybridMetricUpdate {
public:
    explicit HybridMetricUpdate(const Config &config);

    /// Binds the scan to align and the map to align it against. The pointers must
    /// outlive the filter iteration that follows.
    void setFrame(const PointCloud *scan, const std::vector<Mat3> *body_covariances,
                  const VoxelMapType *voxel_map, const EsekfState *kf);

    /// The ESIKF observation model. Install it with
    /// `kf.init_dyn_share(..., [&](auto &s, auto &d) { update(s, d); }, ...)`.
    void operator()(state_ikfom &state, esekfom::dyn_share_datastruct<double> &data);

    struct Stats {
        int plane_matches = 0;
        int point_matches = 0;
        int scan_points = 0;
    };
    const Stats &stats() const { return stats_; }

    /// Correspondences of the last iteration, for publishing and debugging.
    const std::vector<PointToPlaneMatch> &planeMatches() const { return plane_matches_; }
    const std::vector<PointToPointMatch> &pointMatches() const { return point_matches_; }

private:
    /// Lifts the scan into the world frame with its propagated covariance.
    void buildPointsWithCovariance(const state_ikfom &state);
    void findCorrespondences();
    void fillPlaneRows(const state_ikfom &state, esekfom::dyn_share_datastruct<double> &data) const;
    void fillPointRows(const state_ikfom &state, esekfom::dyn_share_datastruct<double> &data) const;

    Config config_;

    const PointCloud *scan_ = nullptr;
    const std::vector<Mat3> *body_covariances_ = nullptr;
    const VoxelMapType *voxel_map_ = nullptr;
    const EsekfState *kf_ = nullptr;

    std::vector<PointWithCov> points_with_cov_;
    std::vector<PointToPlaneMatch> plane_matches_;
    std::vector<PointToPointMatch> point_matches_;
    /// Scan points the plane-only path found no correspondence for.
    std::vector<Vec3> unmatched_points_;
    Stats stats_;

    /// Deviation between the predicted and updated pose, feeding the gate.
    Eigen::Quaterniond deviation_rotation_ = Eigen::Quaterniond::Identity();
    Vec3 deviation_translation_ = Vec3::Zero();
    bool has_pose_deviation_ = false;

public:
    /// Records the pose correction that the last update applied.
    void setPoseDeviation(const Eigen::Quaterniond &rotation, const Vec3 &translation) {
        if (!rotation.coeffs().allFinite() || !translation.allFinite() ||
            !(rotation.norm() > 0.0)) return;
        deviation_rotation_ = rotation.normalized();
        deviation_translation_ = translation;
        has_pose_deviation_ = true;
    }
};

}  // namespace genz_lio
