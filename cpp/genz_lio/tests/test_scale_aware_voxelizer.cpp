// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Behavioural checks on the scale-aware voxelization of paper Sec. IV.
#include "core/ScaleAwareVoxelizer.hpp"
#include "Check.hpp"

#include <cmath>
#include <cstdio>
#include <random>

namespace {

using genz_lio::AdaptiveVoxelizationConfig;
using genz_lio::PointCloud;
using genz_lio::ScaleAwareVoxelizer;

/// A shell of returns at roughly `range` metres, standing in for a scene whose
/// surfaces sit that far from the sensor.
PointCloud makeScan(const double range, const int count, const unsigned seed = 7) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> jitter(0.0, range * 0.05);
    std::uniform_real_distribution<double> angle(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> height(-1.0, 1.0);

    PointCloud cloud;
    cloud.reserve(count);
    for (int i = 0; i < count; ++i) {
        const double theta = angle(rng);
        const double r = range + jitter(rng);
        const double z = height(rng) * range * 0.2;
        genz_lio::Point p;
        p.position = genz_lio::Vec3(r * std::cos(theta), r * std::sin(theta), z);
        p.time_offset = static_cast<float>(i) / static_cast<float>(count) * 0.1f;
        cloud.push_back(p);
    }
    return cloud;
}

void setpointFollowsScale() {
    AdaptiveVoxelizationConfig config;  // paper defaults
    ScaleAwareVoxelizer voxelizer(config, 0.25);

    // A near scene must not ask for more points than a far one.
    ScaleAwareVoxelizer near_voxelizer(config, 0.25);
    const auto near_out = near_voxelizer.process(makeScan(2.0, 20000), 0.1);

    ScaleAwareVoxelizer far_voxelizer(config, 0.25);
    const auto far_out = far_voxelizer.process(makeScan(40.0, 20000), 0.1);

    GENZ_CHECK(near_out.setpoint < far_out.setpoint);
    GENZ_CHECK(near_out.setpoint >= config.min_points);
    GENZ_CHECK(far_out.setpoint <= config.max_points);

    // Beyond the scale threshold the setpoint saturates at N_max.
    ScaleAwareVoxelizer saturated(config, 0.25);
    const auto saturated_out = saturated.process(makeScan(60.0, 20000), 0.1);
    GENZ_CHECK(std::abs(saturated_out.setpoint - config.max_points) < 1e-9);
    GENZ_CHECK(saturated_out.scale_indicator >= config.scale_threshold);
}

void controllerDrivesPointCountTowardSetpoint() {
    AdaptiveVoxelizationConfig config;
    ScaleAwareVoxelizer voxelizer(config, 0.05);  // deliberately far too fine

    const PointCloud scan = makeScan(10.0, 60000);
    ScaleAwareVoxelizer::Output out;
    double first_error = 0.0;
    for (int i = 0; i < 200; ++i) {
        out = voxelizer.process(scan, 0.1 * (i + 1));
        if (i == 0) first_error = std::abs(out.tracking_error);
    }

    // Starting too fine yields far too many points, so the leaf must have grown
    // and the tracking error shrunk.
    GENZ_CHECK(voxelizer.leafSize() > 0.05);
    GENZ_CHECK(std::abs(out.tracking_error) < first_error);
}

void leafSizeStaysWithinBounds() {
    AdaptiveVoxelizationConfig config;
    config.p_gain_max = 1.0;  // absurd gains, to push against the clamps
    config.p_gain_min = 1.0;
    config.d_gain_max = 1.0;
    config.d_gain_min = 1.0;

    ScaleAwareVoxelizer voxelizer(config, 0.25);
    const PointCloud dense = makeScan(10.0, 40000);
    const PointCloud sparse = makeScan(10.0, 50);
    for (int i = 0; i < 20; ++i) {
        voxelizer.process(dense, 0.1 * (i + 1));
        GENZ_CHECK(voxelizer.leafSize() <= ScaleAwareVoxelizer::kMaxLeafSize);
        GENZ_CHECK(voxelizer.leafSize() >= ScaleAwareVoxelizer::kMinLeafSize);
    }
    for (int i = 20; i < 40; ++i) {
        voxelizer.process(sparse, 0.1 * (i + 1));
        GENZ_CHECK(voxelizer.leafSize() <= ScaleAwareVoxelizer::kMaxLeafSize);
        GENZ_CHECK(voxelizer.leafSize() >= ScaleAwareVoxelizer::kMinLeafSize);
    }
}

void mappingCloudIsFinerThanEstimationCloud() {
    AdaptiveVoxelizationConfig config;
    ScaleAwareVoxelizer voxelizer(config, 0.25);
    const auto out = voxelizer.process(makeScan(10.0, 30000), 0.1);
    GENZ_CHECK(out.mapping.size() >= out.estimation.size());

    // Both clouds leave the module ordered by time.
    for (std::size_t i = 1; i < out.estimation.size(); ++i) {
        GENZ_CHECK(out.estimation[i - 1].time_offset <= out.estimation[i].time_offset);
    }
}

void disabledUsesFixedLeafSize() {
    AdaptiveVoxelizationConfig config;
    config.enable = false;
    ScaleAwareVoxelizer voxelizer(config, 0.4);

    const auto out = voxelizer.process(makeScan(10.0, 30000), 0.1);
    GENZ_CHECK(out.leaf_size == 0.4);
    GENZ_CHECK(voxelizer.leafSize() == 0.4);
    GENZ_CHECK(out.mapping.size() == out.estimation.size());

    const auto again = voxelizer.process(makeScan(40.0, 30000), 0.2);
    GENZ_CHECK(again.leaf_size == 0.4);  // scale changes must not move it
}

void firstScanDoesNotDivideByZeroInterval() {
    AdaptiveVoxelizationConfig config;
    ScaleAwareVoxelizer voxelizer(config, 0.25);
    const auto out = voxelizer.process(makeScan(10.0, 20000), 0.0);
    GENZ_CHECK(std::isfinite(out.leaf_size));
    GENZ_CHECK(std::isfinite(voxelizer.leafSize()));
}

}  // namespace

int main() {
    setpointFollowsScale();
    controllerDrivesPointCountTowardSetpoint();
    leafSizeStaysWithinBounds();
    mappingCloudIsFinerThanEstimationCloud();
    disabledUsesFixedLeafSize();
    firstScanDoesNotDivideByZeroInterval();
    std::printf("scale-aware voxelizer ok\n");
    return 0;
}
