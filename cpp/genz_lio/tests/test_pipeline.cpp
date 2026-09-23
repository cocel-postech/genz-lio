// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// End-to-end check: drive a simulated platform through a closed room and see
// whether the pipeline tracks it. The room is bounded in every direction, so the
// scan constrains all three axes.
#include "core/GenZLIO.hpp"
#include "Check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace {

using genz_lio::Config;
using genz_lio::GenZLIO;
using genz_lio::ImuSample;
using genz_lio::RawScan;
using genz_lio::Vec3;

constexpr double kGravity = 9.81;
constexpr double kSpeed = 1.0;         // m/s along +x
constexpr double kScanPeriod = 0.1;    // 10 Hz
constexpr double kImuPeriod = 0.005;   // 200 Hz
constexpr int kRings = 16;
constexpr int kAzimuths = 360;

/// Axis-aligned box, used both for the room and for the pillars inside it.
struct Box {
    Vec3 lower, upper;

    /// Distance at which a ray from `origin` enters the box, or -1 if it misses.
    double entryDistance(const Vec3 &origin, const Vec3 &direction) const {
        double t_near = 0.0, t_far = std::numeric_limits<double>::infinity();
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(direction[axis]) < 1e-9) {
                if (origin[axis] < lower[axis] || origin[axis] > upper[axis]) return -1.0;
                continue;
            }
            double t0 = (lower[axis] - origin[axis]) / direction[axis];
            double t1 = (upper[axis] - origin[axis]) / direction[axis];
            if (t0 > t1) std::swap(t0, t1);
            t_near = std::max(t_near, t0);
            t_far = std::min(t_far, t1);
            if (t_near > t_far) return -1.0;
        }
        return t_near > 1e-6 ? t_near : -1.0;
    }
};

struct Room {
    Vec3 lower{0.0, -4.0, 0.0};
    Vec3 upper{20.0, 4.0, 3.0};

    // Pillars break the translational symmetry of a bare corridor, so that the
    // along-track direction is constrained by nearby structure rather than only
    // by the distant end walls.
    std::vector<Box> pillars{
        {{4.0, -1.0, 0.0}, {4.4, -0.6, 3.0}},
        {{4.0, 0.6, 0.0}, {4.4, 1.0, 3.0}},
        {{7.0, -2.5, 0.0}, {7.4, -2.1, 3.0}},
        {{7.0, 2.1, 0.0}, {7.4, 2.5, 3.0}},
        {{10.0, -1.0, 0.0}, {10.4, -0.6, 3.0}},
    };

    /// Distance from `origin` along `direction` to the wall, or -1 if the ray
    /// somehow escapes.
    double rayLength(const Vec3 &origin, const Vec3 &direction) const {
        double t_max = std::numeric_limits<double>::infinity();
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(direction[axis]) < 1e-9) continue;
            const double bound = direction[axis] > 0 ? upper[axis] : lower[axis];
            t_max = std::min(t_max, (bound - origin[axis]) / direction[axis]);
        }
        if (!std::isfinite(t_max)) return -1.0;

        for (const auto &pillar : pillars) {
            const double t = pillar.entryDistance(origin, direction);
            if (t > 0.0 && t < t_max) t_max = t;
        }
        return t_max;
    }
};

// The platform stands still while the IMU initializes, accelerates over one
// second, then glides. Starting at full speed would leave the filter converging
// from zero velocity, which shows up as a startup offset rather than drift.
constexpr double kRestDuration = 1.0;
constexpr double kRampDuration = 1.0;

double trueSpeed(const double t) {
    if (t < kRestDuration) return 0.0;
    if (t < kRestDuration + kRampDuration) return kSpeed * (t - kRestDuration) / kRampDuration;
    return kSpeed;
}

double trueAcceleration(const double t) {
    const bool ramping = t >= kRestDuration && t < kRestDuration + kRampDuration;
    return ramping ? kSpeed / kRampDuration : 0.0;
}

Vec3 truePosition(const double t) {
    double x = 2.0;
    if (t > kRestDuration) {
        const double ramp = std::min(t - kRestDuration, kRampDuration);
        x += 0.5 * (kSpeed / kRampDuration) * ramp * ramp;
        if (t > kRestDuration + kRampDuration) x += kSpeed * (t - kRestDuration - kRampDuration);
    }
    return Vec3(x, 0.0, 1.5);
}

/// One sweep, expressed in the sensor frame at each point's own measurement time
/// so that the pipeline has real motion distortion to undo.
RawScan makeScan(const Room &room, const double scan_begin) {
    RawScan scan;
    scan.has_time_offsets = true;

    for (int ring = 0; ring < kRings; ++ring) {
        const double elevation = (-15.0 + 30.0 * ring / (kRings - 1)) * M_PI / 180.0;
        for (int a = 0; a < kAzimuths; ++a) {
            const double azimuth = 2.0 * M_PI * a / kAzimuths;
            const double dt = kScanPeriod * a / kAzimuths;

            const Vec3 direction(std::cos(elevation) * std::cos(azimuth),
                                 std::cos(elevation) * std::sin(azimuth),
                                 std::sin(elevation));
            const double range = room.rayLength(truePosition(scan_begin + dt), direction);
            if (range <= 0.0) continue;

            genz_lio::Point p;
            p.position = direction * range;
            p.time_offset = static_cast<float>(dt);
            scan.points.push_back(p);
            scan.rings.push_back(static_cast<std::uint16_t>(ring));
        }
    }
    return scan;
}

/// Specific force and angular rate along the simulated motion profile.
std::vector<ImuSample> makeImu(const double begin, const double end) {
    std::vector<ImuSample> imu;
    for (double t = begin; t <= end + 1e-9; t += kImuPeriod) {
        ImuSample sample;
        sample.timestamp = t;
        // Specific force: the platform's own acceleration plus the reaction to gravity.
        sample.linear_acceleration = Vec3(trueAcceleration(t), 0.0, kGravity);
        sample.angular_velocity = Vec3::Zero();
        imu.push_back(sample);
    }
    return imu;
}

Config makeConfig() {
    Config config;
    // Freeze the original furnished-room regression fixture independently of
    // general-purpose defaults. Keep the existing 0.1 m accuracy threshold.
    config.mapping.voxel_size = 0.5;
    config.mapping.max_layer = 3;
    config.mapping.planar_threshold = 0.01;
    config.hybrid_metric.max_points_per_voxel = 64;
    config.preprocess.lidar_type = genz_lio::LidarType::Velodyne;
    config.preprocess.scan_line = kRings;
    config.preprocess.scan_rate = 10;
    config.preprocess.blind_min = 0.3;
    config.preprocess.blind_max = 50.0;
    return config;
}

void trackASimulatedRun() {
    const Room room;
    Config config = makeConfig();
    GenZLIO pipeline(config);

    double last_error = 0.0;
    int tracked = 0;
    Vec3 last_estimate = Vec3::Zero();

    for (int scan_index = 0; scan_index < 60; ++scan_index) {
        const double begin = scan_index * kScanPeriod;
        const double end = begin + kScanPeriod;

        const auto result =
            pipeline.registerScan(makeScan(room, begin), begin, end, makeImu(begin, end));
        if (!result.valid) continue;

        // The pipeline reports motion relative to wherever it started, so compare
        // displacement rather than absolute position.
        static Vec3 origin = Vec3::Zero();
        static bool origin_set = false;
        if (!origin_set) {
            origin = truePosition(end) - result.pose.translation;
            origin_set = true;
            continue;
        }

        const Vec3 estimate = result.pose.translation + origin;
        last_error = (estimate - truePosition(end)).norm();
        if (const char *v = std::getenv("GENZ_TRACE"))
            std::printf("    scan %2d  err %.4f  vel %.3f %.3f %.3f  planes %4d  points %4d\n",
                        scan_index, last_error, result.velocity.x(), result.velocity.y(),
                        result.velocity.z(), result.plane_matches, result.point_matches);
        last_estimate = estimate;
        ++tracked;

        GENZ_CHECK(result.pose.translation.allFinite());
        GENZ_CHECK(result.voxelized.size() == static_cast<std::size_t>(result.voxelized_points));
        GENZ_CHECK(!result.voxelized.empty());
        GENZ_CHECK(result.voxelized.size() < result.deskewed.size());
        for (const auto &point : result.voxelized) GENZ_CHECK(point.position.allFinite());
    }

    std::printf("  tracked %d scans, final error %.3f m, estimate x=%.2f (true %.2f)\n", tracked,
                last_error, last_estimate.x(), truePosition(60 * kScanPeriod).x());

    GENZ_CHECK(tracked > 30);
    // Roughly five metres of travel through a furnished room.
    GENZ_CHECK(last_error < 0.1);
}

void adaptiveVoxelizationRespondsToTheRoom() {
    const Room room;
    GenZLIO pipeline(makeConfig());

    double first_leaf = 0.0, last_leaf = 0.0;
    for (int scan_index = 0; scan_index < 40; ++scan_index) {
        const double begin = scan_index * kScanPeriod;
        const double end = begin + kScanPeriod;
        const auto result =
            pipeline.registerScan(makeScan(room, begin), begin, end, makeImu(begin, end));
        if (!result.valid || result.leaf_size == 0.0) continue;
        if (first_leaf == 0.0) first_leaf = result.leaf_size;
        last_leaf = result.leaf_size;

        GENZ_CHECK(result.leaf_size >= genz_lio::ScaleAwareVoxelizer::kMinLeafSize);
        GENZ_CHECK(result.leaf_size <= genz_lio::ScaleAwareVoxelizer::kMaxLeafSize);
        GENZ_CHECK(result.scale_indicator > 0.0);
        GENZ_CHECK(result.setpoint >= pipeline.config().adaptive_voxelization.min_points);
    }
    std::printf("  leaf size %.4f -> %.4f m\n", first_leaf, last_leaf);
    GENZ_CHECK(last_leaf > 0.0);
}

void resetClearsTheMap() {
    const Room room;
    GenZLIO pipeline(makeConfig());
    for (int i = 0; i < 5; ++i) {
        const double begin = i * kScanPeriod;
        pipeline.registerScan(makeScan(room, begin), begin, begin + kScanPeriod,
                              makeImu(begin, begin + kScanPeriod));
    }
    GENZ_CHECK(!pipeline.map().empty());

    pipeline.reset();
    GENZ_CHECK(pipeline.map().empty());
    GENZ_CHECK(!pipeline.initialized());
}

}  // namespace

int main() {
    trackASimulatedRun();
    adaptiveVoxelizationRespondsToTheRoom();
    resetClearsTheMap();
    std::printf("pipeline ok\n");
    return 0;
}
