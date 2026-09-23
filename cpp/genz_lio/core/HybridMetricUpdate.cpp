// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "HybridMetricUpdate.hpp"

#include "SO3Math.hpp"
#include "StateTransforms.hpp"
#include "VoxelPrunedSearch.hpp"

#include <algorithm>
#include <cmath>

namespace genz_lio {
namespace {

/// The filter stores inverse covariances; keep them in a range the solver can
/// work with, so that a near-zero covariance cannot dominate the update.
constexpr double kMinInverseCovariance = 1e-5;
constexpr double kMaxInverseCovariance = 1e5;

/// A point-to-point residual shorter than this has no usable direction: the
/// query point and its match coincide, so residual / |residual| is 0 / 0.
constexpr double kMinResidualNorm = 1e-9;

/// Zeroes one measurement row so it contributes nothing to H^T R^-1 H or to the
/// residual, which is what a degenerate correspondence is worth. Dropping the
/// row outright would mean resizing inside the parallel fill.
void silenceRow(esekfom::dyn_share_datastruct<double> &data, const int row) {
    data.h_x.block<1, 12>(row, 0).setZero();
    data.h(row) = 0.0;
    data.R(row) = kMinInverseCovariance;
}

}  // namespace

HybridMetricUpdate::HybridMetricUpdate(const Config &config) : config_(config) {}

void HybridMetricUpdate::setFrame(const PointCloud *scan,
                                  const std::vector<Mat3> *body_covariances,
                                  const VoxelMapType *voxel_map, const EsekfState *kf) {
    scan_ = scan;
    body_covariances_ = body_covariances;
    voxel_map_ = voxel_map;
    kf_ = kf;
}

void HybridMetricUpdate::buildPointsWithCovariance(const state_ikfom &state) {
    points_with_cov_.resize(scan_->size());
    for (std::size_t i = 0; i < scan_->size(); ++i) {
        PointWithCov &pv = points_with_cov_[i];
        pv.point = (*scan_)[i].position;
        pv.point_world = transformPointToWorld(state, pv.point);
        pv.cov_lidar = (*body_covariances_)[i];
        pv.cov = transformCovarianceToWorld(pv.point, *kf_, pv.cov_lidar);
    }
}

void HybridMetricUpdate::findCorrespondences() {
    plane_matches_.clear();
    point_matches_.clear();

    const auto &hybrid = config_.hybrid_metric;
    const auto &mapping = config_.mapping;

    if (!hybrid.enable) {
        // Point-to-plane only: the paper's baseline ablation.
        unmatched_points_.clear();
        buildPlaneResidualList(*voxel_map_, mapping.voxel_size, mapping.sigma_num,
                               mapping.max_layer, points_with_cov_, plane_matches_,
                               unmatched_points_);
        return;
    }

    // Each point carries its own gate, scaled by its range.
    buildHybridResidualListPerPointRange(
        *voxel_map_, mapping.voxel_size, mapping.sigma_num, hybrid.sigma_num,
        hybrid.adaptive_threshold.initial_threshold, hybrid.adaptive_threshold.max_range_motion,
        deviation_rotation_, deviation_translation_, has_pose_deviation_ ? 2 : 1,
        mapping.max_layer, points_with_cov_,
        plane_matches_, point_matches_);
}

void HybridMetricUpdate::fillPlaneRows(const state_ikfom &state,
                                       esekfom::dyn_share_datastruct<double> &data) const {
    const int plane_count = static_cast<int>(plane_matches_.size());
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < plane_count; ++i) {
        const PointToPlaneMatch &match = plane_matches_[i];

        const Vec3 point_lidar = match.point;
        const Vec3 point_body = state.offset_R_L_I * point_lidar + state.offset_T_L_I;
        const Mat3 lidar_cross = skewSymmetric<double>(point_lidar);
        const Mat3 body_cross = skewSymmetric<double>(point_body);
        const Vec3 normal = match.normal;

        // Measurement Jacobian of the point-to-plane distance.
        const Vec3 C(state.rot.conjugate() * normal);
        const Vec3 A(body_cross * C);
        if (config_.mapping.extrinsic_est_en) {
            const Vec3 B(lidar_cross * state.offset_R_L_I.conjugate() * C);
            data.h_x.block<1, 12>(i, 0) << normal.x(), normal.y(), normal.z(), A[0], A[1], A[2],
                B[0], B[1], B[2], C[0], C[1], C[2];
        } else {
            data.h_x.block<1, 12>(i, 0) << normal.x(), normal.y(), normal.z(), A[0], A[1], A[2],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        data.h(i) = -(normal.dot(match.point_world) + match.d);

        // Residual covariance: the plane's own uncertainty projected onto the
        // normal, plus the point's measurement uncertainty along the same direction.
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = match.point_world - match.center;
        J_nq.block<1, 3>(0, 3) = -match.normal;
        const double sigma_plane = J_nq * match.plane_cov * J_nq.transpose();

        const Mat3 cov_world = state.rot * state.offset_R_L_I * match.cov_lidar *
                               state.offset_R_L_I.conjugate() * state.rot.conjugate();
        const double inverse_cov = 1.0 / (sigma_plane + normal.transpose() * cov_world * normal);
        if (!std::isfinite(inverse_cov)) {
            silenceRow(data, i);
            continue;
        }
        data.R(i) = std::clamp(inverse_cov, kMinInverseCovariance, kMaxInverseCovariance);
    }
}

void HybridMetricUpdate::fillPointRows(const state_ikfom &state,
                                       esekfom::dyn_share_datastruct<double> &data) const {
    const int plane_count = static_cast<int>(plane_matches_.size());
    const int point_count = static_cast<int>(point_matches_.size());
    const double lambda_po = config_.hybrid_metric.lambda_po;
    const double root_voxel_size = config_.mapping.voxel_size;

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int j = 0; j < point_count; ++j) {
        const PointToPointMatch &match = point_matches_[j];
        const int row = plane_count + j;

        const Vec3 source_lidar = match.point;
        const Vec3 source_body = state.offset_R_L_I * source_lidar + state.offset_T_L_I;
        const Vec3 source_world = state.rot * source_body + state.pos;
        const Vec3 residual = source_world - match.point_center;
        const double residual_norm = residual.norm();
        if (!(residual_norm > kMinResidualNorm)) {
            // Exactly-coincident points do happen once the adaptive leaf size
            // drops far below the map resolution, and dividing by the norm here
            // used to poison the whole update with NaN.
            silenceRow(data, row);
            continue;
        }
        const Vec3 direction = residual / residual_norm;

        // Measurement Jacobian of the scalar residual along its own direction.
        const Vec3 C(state.rot.conjugate() * direction);
        const Vec3 A(skewSymmetric<double>(source_body) * C);
        if (config_.mapping.extrinsic_est_en) {
            const Vec3 B(skewSymmetric<double>(source_lidar) * state.offset_R_L_I.conjugate() * C);
            data.h_x.block<1, 12>(row, 0) << direction.x(), direction.y(), direction.z(), A[0],
                A[1], A[2], B[0], B[1], B[2], C[0], C[1], C[2];
        } else {
            data.h_x.block<1, 12>(row, 0) << direction.x(), direction.y(), direction.z(), A[0],
                A[1], A[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        data.h(row) = -residual_norm;

        // R_po^norm: query and target uncertainty projected onto the residual.
        const Mat3 cov_world = state.rot * state.offset_R_L_I * match.cov_lidar *
                                   state.offset_R_L_I.conjugate() * state.rot.conjugate() +
                               match.point_cov;
        const double sigma_norm = (direction.transpose() * cov_world * direction).value();

        // R_disc: how coarsely the map resolves this correspondence. A search that
        // had to open many voxels for few neighbours resolves the target poorly.
        const double sigma_disc = match.num_neighbors > 0
                                      ? static_cast<double>(match.num_candidate_voxels) *
                                            root_voxel_size * root_voxel_size /
                                            static_cast<double>(match.num_neighbors)
                                      : 0.0;

        // Eq. (30): R_comb = lambda_po * (R_norm + R_disc). lambda_po offsets the
        // difference in scale against the point-to-plane covariance, which is
        // fitted from many points rather than one correspondence.
        const double combined_cov = lambda_po * (sigma_norm + sigma_disc);
        const double inverse_cov = 1.0 / combined_cov;
        if (!std::isfinite(inverse_cov)) {
            silenceRow(data, row);
            continue;
        }
        data.R(row) = std::clamp(inverse_cov, kMinInverseCovariance, kMaxInverseCovariance);
    }
}

void HybridMetricUpdate::operator()(state_ikfom &state,
                                    esekfom::dyn_share_datastruct<double> &data) {
    buildPointsWithCovariance(state);
    findCorrespondences();

    const int plane_count = static_cast<int>(plane_matches_.size());
    const int point_count = static_cast<int>(point_matches_.size());
    const int total = plane_count + point_count;

    stats_.plane_matches = plane_count;
    stats_.point_matches = point_count;
    stats_.scan_points = static_cast<int>(scan_->size());

    if (total < 1) {
        data.valid = false;
        return;
    }

    data.h_x = Eigen::MatrixXd::Zero(total, 12);
    data.h.resize(total);
    data.R.resize(total, 1);

    fillPlaneRows(state, data);
    if (point_count > 0) fillPointRows(state, data);
}

}  // namespace genz_lio
