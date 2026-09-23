// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "Preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace genz_lio {
namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
/// Degrees swept per second, per hertz of scan rate: 360 deg / 1000 ms.
constexpr double kDegreesPerMillisecondPerHz = 0.361;
/// Below this, a coordinate counts as an empty return rather than a measurement.
constexpr double kMinCoordinate = 1e-7;

}  // namespace

bool Preprocessor::reconstructTimeOffsets(RawScan &scan) const {
    if (scan.rings.size() != scan.points.size()) return false;

    const int scan_lines = config_.scan_line;
    const double degrees_per_ms = kDegreesPerMillisecondPerHz * config_.scan_rate;
    if (degrees_per_ms <= 0.0 || scan_lines <= 0) return false;

    std::vector<bool> is_first(scan_lines, true);
    std::vector<double> first_yaw(scan_lines, 0.0);
    std::vector<double> last_time(scan_lines, 0.0);

    for (std::size_t i = 0; i < scan.points.size(); ++i) {
        const int ring = scan.rings[i];
        if (ring < 0 || ring >= scan_lines || !scan.points[i].position.allFinite()) {
            scan.points[i].time_offset = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        Point &point = scan.points[i];
        const double yaw = std::atan2(point.position.y(), point.position.x()) * kRadToDeg;

        if (is_first[ring]) {
            first_yaw[ring] = yaw;
            is_first[ring] = false;
            point.time_offset = 0.0f;
            last_time[ring] = 0.0;
            continue;
        }

        // Time since this ring's first return, from how far the beam has swept.
        double offset_ms = (yaw <= first_yaw[ring]) ? (first_yaw[ring] - yaw) / degrees_per_ms
                                                    : (first_yaw[ring] - yaw + 360.0) / degrees_per_ms;
        // A yaw that has wrapped past the start belongs to the next revolution.
        if (offset_ms < last_time[ring]) offset_ms += 360.0 / degrees_per_ms;

        last_time[ring] = offset_ms;
        point.time_offset = static_cast<float>(offset_ms * 1e-3);
    }
    return true;
}

bool Preprocessor::prepareTiming(RawScan &scan) const {
    if (scan.timing_prepared) return !scan.points.empty();
    const bool reported = scan.has_time_offsets;
    if (!reported && !reconstructTimeOffsets(scan)) return false;
    const bool guard = reported && config_.lidar_type == LidarType::Velodyne;
    const double period = config_.scan_rate > 0 ? 1.0 / config_.scan_rate : 0.1;
    const bool rings_present = scan.rings.size() == scan.points.size();
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -minimum;
    std::size_t kept = 0;
    for (std::size_t i = 0; i < scan.points.size(); ++i) {
        const auto &point = scan.points[i];
        const double t = point.time_offset;
        if (!point.position.allFinite() || !std::isfinite(t)) continue;
        if (guard && (std::abs(t) < period / 1800.0 || std::abs(t) > period * 1.1)) continue;
        minimum = std::min(minimum, t);
        maximum = std::max(maximum, t);
        scan.points[kept] = point;
        if (rings_present) scan.rings[kept] = scan.rings[i];
        ++kept;
    }
    scan.points.resize(kept);
    if (rings_present) scan.rings.resize(kept);
    else scan.rings.clear();
    if (kept == 0) return false;
    scan.time_min = minimum;
    scan.time_max = maximum;
    scan.has_time_offsets = true;
    scan.timing_prepared = true;
    return true;
}

PointCloud Preprocessor::process(const RawScan &raw) const {
    RawScan scan = raw;
    if (!scan.has_time_offsets) reconstructTimeOffsets(scan);

    const double blind_min_sq = config_.blind_min * config_.blind_min;
    const double blind_max_sq = config_.blind_max * config_.blind_max;
    const int filter_num = std::max(1, config_.point_filter_num);

    // Velodyne drivers occasionally emit returns whose stamps do not belong to
    // this sweep — the first column carries near-zero stamps, and retransmitted
    // packets carry stamps from a neighbouring revolution. Either would distort
    // the scan end time that IMU propagation keys off, so both are dropped.
    const bool guard_timestamps =
        config_.lidar_type == LidarType::Velodyne && scan.has_time_offsets && !scan.timing_prepared;
    const double scan_period = config_.scan_rate > 0 ? 1.0 / config_.scan_rate : 0.1;
    const double min_valid_time = scan_period / 1800.0;
    const double max_valid_time = scan_period * 1.1;

    // Offsets are reported relative to whichever end of the sweep the driver
    // stamps; rebasing onto the earliest return makes both conventions the same
    // downstream, and keeps deskewing aligned with the IMU poses.
    const double time_origin = scan.has_time_offsets ? scan.time_min : 0.0;

    PointCloud out;
    out.reserve(scan.points.size() / filter_num + 1);

    int valid_index = 0;
    for (const auto &point : scan.points) {
        if (!point.position.allFinite() || !std::isfinite(point.time_offset)) continue;
        if (guard_timestamps) {
            const double t = std::abs(point.time_offset);
            if (t < min_valid_time || t > max_valid_time) continue;
        }

        // Decimation counts points that reached this far rather than raw
        // indices, so a run of rejected returns cannot shift which points are
        // kept. The two agree at the default filter_num of 1.
        if (valid_index++ % filter_num != 0) continue;

        if (point.position.cwiseAbs().maxCoeff() < kMinCoordinate) continue;

        const double range_sq = point.position.squaredNorm();
        if (range_sq <= blind_min_sq || range_sq >= blind_max_sq) continue;

        Point rebased = point;
        // The offsets are floats and time_origin is the double they were reduced
        // to, so the earliest point can land a few nanoseconds below zero. Clamp,
        // or deskewing sees an offset that predates the sweep it belongs to.
        rebased.time_offset =
            static_cast<float>(std::max(0.0, point.time_offset - time_origin));
        out.push_back(rebased);
    }
    return out;
}

}  // namespace genz_lio
