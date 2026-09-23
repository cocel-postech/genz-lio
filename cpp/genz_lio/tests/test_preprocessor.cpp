// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "core/Preprocessor.hpp"
#include "Check.hpp"

#include <cmath>
#include <cstdio>

namespace {

using genz_lio::LidarType;
using genz_lio::PreprocessConfig;
using genz_lio::Preprocessor;
using genz_lio::RawScan;
using genz_lio::Vec3;

genz_lio::Point makePoint(const Vec3 &position, const float time = 0.01f) {
    genz_lio::Point p;
    p.position = position;
    p.time_offset = time;
    return p;
}

void gatesByRange() {
    PreprocessConfig config;
    config.lidar_type = LidarType::Ouster;
    config.blind_min = 1.0;
    config.blind_max = 50.0;

    RawScan scan;
    scan.points = {
        makePoint(Vec3(0.5, 0.0, 0.0)),   // inside the blind radius
        makePoint(Vec3(10.0, 0.0, 0.0)),  // kept
        makePoint(Vec3(80.0, 0.0, 0.0)),  // beyond the far limit
        makePoint(Vec3(0.0, 0.0, 0.0)),   // empty return
    };

    const auto out = Preprocessor(config).process(scan);
    GENZ_CHECK(out.size() == 1);
    GENZ_CHECK(std::abs(out[0].position.x() - 10.0) < 1e-12);
}

void decimatesByFilterNumber() {
    PreprocessConfig config;
    config.lidar_type = LidarType::Ouster;
    config.blind_min = 0.1;
    config.blind_max = 100.0;
    config.point_filter_num = 3;

    RawScan scan;
    for (int i = 0; i < 30; ++i) scan.points.push_back(makePoint(Vec3(5.0 + i, 0.0, 0.0)));

    GENZ_CHECK(Preprocessor(config).process(scan).size() == 10);

    config.point_filter_num = 1;
    GENZ_CHECK(Preprocessor(config).process(scan).size() == 30);
}

void velodyneDropsStampsOutsideTheSweep() {
    PreprocessConfig config;
    config.lidar_type = LidarType::Velodyne;
    config.scan_rate = 10;  // 0.1 s sweep
    config.blind_min = 0.1;
    config.blind_max = 100.0;

    RawScan scan;
    scan.points = {
        makePoint(Vec3(5.0, 0.0, 0.0), 1e-6f),   // below 0.1/1800, first column
        makePoint(Vec3(6.0, 0.0, 0.0), 0.05f),   // mid sweep, kept
        makePoint(Vec3(7.0, 0.0, 0.0), 0.2f),    // beyond 1.1 * sweep, retransmitted
        makePoint(Vec3(8.0, 0.0, 0.0), 0.099f),  // end of sweep, kept
    };

    const auto out = Preprocessor(config).process(scan);
    GENZ_CHECK(out.size() == 2);
    GENZ_CHECK(std::abs(out[0].position.x() - 6.0) < 1e-12);
    GENZ_CHECK(std::abs(out[1].position.x() - 8.0) < 1e-12);

    // The same guard must not fire for a sensor that reports its own timing.
    config.lidar_type = LidarType::Ouster;
    GENZ_CHECK(Preprocessor(config).process(scan).size() == 4);
}

void reconstructsTimeFromAzimuth() {
    PreprocessConfig config;
    config.lidar_type = LidarType::Velodyne;
    config.scan_line = 2;
    config.scan_rate = 10;
    config.blind_min = 0.1;
    config.blind_max = 100.0;

    // One ring swept clockwise: yaw decreases, so time must increase.
    RawScan scan;
    scan.has_time_offsets = false;
    for (int i = 0; i < 8; ++i) {
        const double yaw = -i * (M_PI / 8.0);
        scan.points.push_back(makePoint(Vec3(10.0 * std::cos(yaw), 10.0 * std::sin(yaw), 0.0), 0.0f));
        scan.rings.push_back(0);
    }

    const auto out = Preprocessor(config).process(scan);
    GENZ_CHECK(out.size() == 8);
    GENZ_CHECK(out.front().time_offset == 0.0f);  // first return of the ring
    for (std::size_t i = 1; i < out.size(); ++i) {
        GENZ_CHECK(out[i].time_offset > out[i - 1].time_offset);
    }
    // A full sweep at 10 Hz lasts 0.1 s, so a half turn must stay under that.
    GENZ_CHECK(out.back().time_offset < 0.1f);
}

void missingRingsLeaveTimingUntouched() {
    PreprocessConfig config;
    config.lidar_type = LidarType::Velodyne;
    config.blind_min = 0.1;
    config.blind_max = 100.0;

    RawScan scan;
    scan.has_time_offsets = false;  // but no ring indices supplied
    scan.points = {makePoint(Vec3(5.0, 0.0, 0.0), 0.02f)};

    const auto out = Preprocessor(config).process(scan);
    GENZ_CHECK(out.size() == 1);
    GENZ_CHECK(std::abs(out[0].time_offset - 0.02f) < 1e-9f);
}

void rebasesTimesReportedAgainstTheSweepEnd() {
    // Some Velodyne platforms stamp the message at the end of the sweep and
    // report times running back to its start, so every offset is negative. They
    // must come out as offsets from the first return, or deskewing silently
    // stops applying: the IMU poses run forward from zero and no point ever
    // matches them.
    PreprocessConfig config;
    config.lidar_type = LidarType::Velodyne;
    config.scan_rate = 10;
    config.blind_min = 0.1;
    config.blind_max = 100.0;

    RawScan scan;
    scan.time_min = -0.0982;
    scan.time_max = 0.0013;
    for (int i = 0; i < 10; ++i) {
        const double t = scan.time_min + i * (scan.time_max - scan.time_min) / 9.0;
        scan.points.push_back(makePoint(Vec3(5.0 + i, 0.0, 0.0), static_cast<float>(t)));
    }

    const auto out = Preprocessor(config).process(scan);
    GENZ_CHECK(!out.empty());
    for (const auto &p : out) {
        GENZ_CHECK(p.time_offset >= 0.0f);
        GENZ_CHECK(p.time_offset <= static_cast<float>(scan.time_max - scan.time_min) + 1e-6f);
    }
    for (std::size_t i = 1; i < out.size(); ++i) {
        GENZ_CHECK(out[i].time_offset > out[i - 1].time_offset);  // order preserved
    }

    // The forward convention must be left exactly as it is.
    RawScan forward;
    forward.time_min = 0.0;
    forward.time_max = 0.09;
    forward.points.push_back(makePoint(Vec3(5.0, 0.0, 0.0), 0.05f));
    const auto kept = Preprocessor(config).process(forward);
    GENZ_CHECK(kept.size() == 1);
    GENZ_CHECK(std::abs(kept[0].time_offset - 0.05f) < 1e-6f);
}

}  // namespace

int main() {
    gatesByRange();
    decimatesByFilterNumber();
    velodyneDropsStampsOutsideTheSweep();
    rebasesTimesReportedAgainstTheSweepEnd();
    reconstructsTimeFromAzimuth();
    missingRingsLeaveTimingUntouched();
    std::printf("preprocessor ok\n");
    return 0;
}
