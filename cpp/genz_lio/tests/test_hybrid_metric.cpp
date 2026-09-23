// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Checks the adaptive residual gate and the hybrid-metric observation model of
// paper Sec. V-E.
#include "core/HybridMetricUpdate.hpp"

#include "core/VoxelMap.hpp"

#include "core/StateTransforms.hpp"
#include "Check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

namespace {

using genz_lio::Config;
using genz_lio::HybridMetricUpdate;
using genz_lio::Mat3;
using genz_lio::PointWithCov;
using genz_lio::Vec3;

/// A map of a flat floor plus a wall, so that both planar and non-planar
/// structure is present.
genz_lio::VoxelMapType makeMap(std::vector<PointWithCov> &storage) {
    std::mt19937 rng(11);
    std::normal_distribution<double> noise(0.0, 0.002);

    for (double x = -6.0; x < 6.0; x += 0.05) {
        for (double y = -6.0; y < 6.0; y += 0.05) {
            PointWithCov pv;
            pv.point = Vec3(x, y, noise(rng));
            pv.point_world = pv.point;
            pv.cov = Mat3::Identity() * 1e-4;
            pv.cov_lidar = pv.cov;
            storage.push_back(pv);
        }
    }
    for (double y = -6.0; y < 6.0; y += 0.05) {
        for (double z = 0.0; z < 3.0; z += 0.05) {
            PointWithCov pv;
            pv.point = Vec3(6.0 + noise(rng), y, z);
            pv.point_world = pv.point;
            pv.cov = Mat3::Identity() * 1e-4;
            pv.cov_lidar = pv.cov;
            storage.push_back(pv);
        }
    }

    genz_lio::VoxelMapType map;
    const Config config;
    buildVoxelMap(storage, config.mapping.voxel_size, config.mapping.max_layer,
                  config.mapping.layer_point_size, config.mapping.max_points_size,
                  config.mapping.max_mature_points_size, config.mapping.planar_threshold,
                  config.hybrid_metric.max_points_per_voxel,
                  config.hybrid_metric.reduction_ratio, config.hybrid_metric.enable, map);
    return map;
}

genz_lio::EsekfState makeFilter() {
    genz_lio::EsekfState kf;
    static double limit[23];
    std::fill(limit, limit + 23, 0.001);
    kf.init_dyn_share(genz_lio::getF, genz_lio::dfDx, genz_lio::dfDw,
                      [](genz_lio::state_ikfom &, esekfom::dyn_share_datastruct<double> &d) {
                          d.valid = false;
                      },
                      1, limit);
    return kf;
}

void observationModelProducesUsableRows() {
    std::vector<PointWithCov> storage;
    auto map = makeMap(storage);
    GENZ_CHECK(!map.empty());

    // Re-observe part of the floor and the wall from the origin.
    genz_lio::PointCloud scan;
    std::vector<Mat3> body_covariances;
    for (double x = -3.0; x < 3.0; x += 0.2) {
        for (double y = -3.0; y < 3.0; y += 0.2) {
            genz_lio::Point p;
            p.position = Vec3(x, y, 0.0);
            scan.push_back(p);
            body_covariances.push_back(Mat3::Identity() * 1e-4);
        }
    }

    Config config;
    genz_lio::EsekfState kf = makeFilter();
    HybridMetricUpdate update(config);
    update.setFrame(&scan, &body_covariances, &map, &kf);

    genz_lio::state_ikfom state = kf.get_x();
    esekfom::dyn_share_datastruct<double> data;
    data.valid = true;
    update(state, data);

    GENZ_CHECK(data.valid);
    const auto &stats = update.stats();
    GENZ_CHECK(stats.plane_matches > 0);  // a floor this flat must yield plane matches
    GENZ_CHECK(stats.scan_points == static_cast<int>(scan.size()));

    const int rows = stats.plane_matches + stats.point_matches;
    GENZ_CHECK(data.h.size() == rows);
    GENZ_CHECK(data.h_x.rows() == rows && data.h_x.cols() == 12);
    GENZ_CHECK(data.h.allFinite() && data.h_x.allFinite() && data.R.allFinite());
    for (int i = 0; i < rows; ++i) {
        GENZ_CHECK(data.R(i) >= 1e-5 && data.R(i) <= 1e5);
    }

    for (auto &entry : map) delete entry.second;
}

void emptyMapInvalidatesTheUpdate() {
    genz_lio::VoxelMapType empty_map;
    genz_lio::PointCloud scan(10);
    std::vector<Mat3> covariances(10, Mat3::Identity() * 1e-4);
    for (std::size_t i = 0; i < scan.size(); ++i) scan[i].position = Vec3(i * 0.5, 0.0, 0.0);

    Config config;
    genz_lio::EsekfState kf = makeFilter();
    HybridMetricUpdate update(config);
    update.setFrame(&scan, &covariances, &empty_map, &kf);

    genz_lio::state_ikfom state = kf.get_x();
    esekfom::dyn_share_datastruct<double> data;
    data.valid = true;
    update(state, data);
    GENZ_CHECK(!data.valid);
}

// Fractional root sizes used by the experiment configs must retain a physical
// spacing between candidates. An integer root size silently made that spacing
// zero below one metre, so duplicate returns exhausted the candidate budget.
void fractionalVoxelsKeepDistinctCandidates() {
    for (const double root_size : {0.25, 0.5, 1.0, 1.5, 2.0}) {
        for (const int layer : {0, 1, 2}) {
            genz_lio::OctoTree cell(3, layer, {5, 5, 5}, 100, 100, 0.01f,
                                   root_size, 64, 4, true);
            // With a budget divided by four per layer, each halving of the
            // edge retains the root's spacing: root_size / sqrt(64).
            const double spacing = root_size / 8.0;
            PointWithCov first{};
            first.point = Vec3::Zero();
            PointWithCov near = first;
            near.point.x() = 0.25 * spacing;
            PointWithCov far = first;
            far.point.x() = 1.5 * spacing;

            cell.temp_points_ = {first, first, near, far};
            cell.initCandidatePoints();
            GENZ_CHECK(cell.grid_points_.size() == 2);
            GENZ_CHECK((cell.grid_points_[1].point - far.point).norm() < 1e-12);

            for (int repeat = 0; repeat < 80; ++repeat) {
                cell.updateCandidatePoint(first);
                cell.updateCandidatePoint(near);
            }
            GENZ_CHECK(cell.grid_points_.size() == 2);
            PointWithCov another = first;
            another.point.y() = 1.5 * spacing;
            cell.updateCandidatePoint(another);
            GENZ_CHECK(cell.grid_points_.size() == 3);
        }
    }
}

// computeBodyCovariance built its tangent basis as (1, 1, -(dx + dy) / dz),
// which divides by zero for any point lying exactly in the sensor's z = 0 plane.
// Livox scan patterns emit those (0.6% of Mid-70 returns), and the NaN covariance
// it produced propagated through R into the ESIKF and killed the run.
void zeroElevationPointsKeepAFiniteCovariance() {
    const float range_inc = 0.04f;
    const float degree_inc = 0.05f;

    const Vec3 degenerate[] = {
        Vec3(3.0, 0.0, 0.0),    // straight ahead, exactly on the horizon
        Vec3(3.0, 4.0, 0.0),    // off-axis but still exactly on the horizon
        Vec3(-2.0, 2.0, 0.0),   // dx + dy == 0 as well, so the old numerator vanished
        Vec3(0.0, 5.0, 0.0),
    };
    for (const Vec3 &point : degenerate) {
        Vec3 mutable_point = point;
        const Mat3 cov = genz_lio::computeBodyCovariance(mutable_point, range_inc, degree_inc);
        GENZ_CHECK(cov.allFinite());
    }

    // Where the reference was well defined the new basis must agree with it: the
    // covariance only depends on the plane normal to the direction, not on which
    // orthonormal basis spans it.
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> spread(-4.0, 4.0);
    for (int i = 0; i < 500; ++i) {
        Vec3 point(spread(rng), spread(rng), spread(rng));
        if (std::abs(point.z()) < 0.2 || point.norm() < 0.5) continue;

        Vec3 mutable_point = point;
        const Mat3 actual = genz_lio::computeBodyCovariance(mutable_point, range_inc, degree_inc);

        // The reference construction, spelled out.
        const double range = point.norm();
        Vec3 direction = point / range;
        Mat3 direction_hat;
        direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0),
            -direction(1), direction(0), 0;
        Vec3 b1(1, 1, -(direction(0) + direction(1)) / direction(2));
        b1.normalize();
        Vec3 b2 = b1.cross(direction);
        b2.normalize();
        Eigen::Matrix<double, 3, 2> N;
        N << b1(0), b2(0), b1(1), b2(1), b1(2), b2(2);
        Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
        const double angular = std::pow(std::sin(degree_inc * M_PI / 180.0), 2);
        const Mat3 expected = direction * (range_inc * range_inc) * direction.transpose() +
                              A * (angular * Eigen::Matrix2d::Identity()) * A.transpose();

        GENZ_CHECK((actual - expected).cwiseAbs().maxCoeff() < 1e-9);
    }
}

}  // namespace

int main() {
    observationModelProducesUsableRows();
    emptyMapInvalidatesTheUpdate();
    fractionalVoxelsKeepDistinctCandidates();
    zeroElevationPointsKeepAFiniteCovariance();
    std::printf("hybrid metric ok\n");
    return 0;
}
