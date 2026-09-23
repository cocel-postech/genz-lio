// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Checks that the core headers stand on their own — no ROS, no PCL — and that
// the defaults match the shipped general-purpose configuration.
#include "core/Config.hpp"
#include "core/SO3Math.hpp"
#include "core/StateIkfom.hpp"
#include "core/Types.hpp"
#include "Check.hpp"

#include <cmath>
#include <cstdio>

namespace {

void checkDefaults() {
    const genz_lio::Config cfg;
    const auto &av = cfg.adaptive_voxelization;
    GENZ_CHECK(av.window_size == 5);            // N_w
    GENZ_CHECK(av.scale_threshold == 30.0);     // tau_m
    GENZ_CHECK(av.setpoint_exponent == 2);      // p
    GENZ_CHECK(av.min_points == 1000);          // N_min
    GENZ_CHECK(av.max_points == 4000);          // N_max
    GENZ_CHECK(av.error_sensitivity == 0.1);    // lambda_p
    GENZ_CHECK(av.error_rate_sensitivity == 0.2);  // lambda_d
    GENZ_CHECK(av.p_gain_min == 5e-6 && av.p_gain_max == 5e-5);
    GENZ_CHECK(av.d_gain_min == 5e-8 && av.d_gain_max == 5e-7);

    GENZ_CHECK(cfg.hybrid_metric.enable);
    GENZ_CHECK(cfg.hybrid_metric.lambda_po == 0.05);
    GENZ_CHECK(cfg.mapping.voxel_size == 1.0);  // d_root
    GENZ_CHECK(cfg.mapping.max_layer == 4);
    GENZ_CHECK(cfg.mapping.planar_threshold == 0.001);
    GENZ_CHECK(cfg.mapping.max_points_size == 100);
    GENZ_CHECK(cfg.mapping.max_mature_points_size == 100);
    GENZ_CHECK(cfg.hybrid_metric.max_points_per_voxel == 128);
    GENZ_CHECK(cfg.hybrid_metric.reduction_ratio == 4);
}

void checkSO3() {
    const genz_lio::Vec3 v(0.1, 0.2, 0.3);  // norm below pi, so exp/log round-trips
    const genz_lio::Mat3 R = genz_lio::expSO3<double>(v);
    GENZ_CHECK(std::abs(R.determinant() - 1.0) < 1e-12);
    GENZ_CHECK((R * R.transpose() - genz_lio::Mat3::Identity()).norm() < 1e-12);
    GENZ_CHECK((genz_lio::logSO3<double>(R) - v).norm() < 1e-12);

    const genz_lio::Mat3 skew = genz_lio::skewSymmetric<double>(v);
    GENZ_CHECK((skew + skew.transpose()).norm() < 1e-15);
    GENZ_CHECK((skew * v).norm() < 1e-15);
}

void checkVoxelKey() {
    const genz_lio::VoxelKey key(genz_lio::Vec3(-0.5, 2.7, 3.0));
    GENZ_CHECK(key.x == -1 && key.y == 2 && key.z == 3);
    GENZ_CHECK(key == genz_lio::VoxelKey(-1, 2, 3));

    const genz_lio::VoxelHash hash;
    GENZ_CHECK(hash(key) == hash(genz_lio::VoxelKey(-1, 2, 3)));
}

void checkFilterModel() {
    genz_lio::state_ikfom s;
    genz_lio::input_ikfom in;
    in.acc = genz_lio::Vect3(Eigen::Vector3d(0.0, 0.0, 9.81));

    const auto f = genz_lio::getF(s, in);
    GENZ_CHECK(f.allFinite());
    GENZ_CHECK(genz_lio::dfDx(s, in).allFinite());
    GENZ_CHECK(genz_lio::dfDw(s, in).allFinite());
    GENZ_CHECK(genz_lio::processNoiseCov().allFinite());
}

}  // namespace

int main() {
    checkDefaults();
    checkSO3();
    checkVoxelKey();
    checkFilterModel();
    std::printf("core types ok\n");
    return 0;
}
