// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// PointCloud2 to RawScan.
//
// Rather than registering a point struct per vendor, the fields are read by name
// and datatype straight out of the message. Every driver the paper's benchmark
// touches — Velodyne, Ouster, Hesai, RoboSense, Livox published as PointCloud2 —
// has a timing field, unit, and relative/absolute convention decoded here.
#pragma once

#include "RosCompat.hpp"
#include "core/Config.hpp"
#include "core/Preprocessor.hpp"

#include <string>

namespace genz_lio {
namespace ros_wrapper {

struct ConvertedScan {
    RawScan scan;
    /// Span between the earliest and latest reported time, in seconds.
    double duration = 0.0;
    /// What the conversion had to fall back on, empty when nothing was amiss.
    std::string warning;
};

/// Reads seconds relative to msg.header.stamp into a RawScan. Sensor-specific
/// timestamp schemas include LivoxPcl absolute nanoseconds and Hesai/Robosense
/// absolute seconds. ScanBuffer validates timing before selecting IMU samples.
ConvertedScan toRawScan(const PointCloud2 &msg, LidarType lidar_type);

}  // namespace ros_wrapper
}  // namespace genz_lio
