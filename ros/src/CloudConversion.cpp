// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "CloudConversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace genz_lio {
namespace ros_wrapper {
namespace {

/// Names a driver might give the per-point timing field, in the order we prefer
/// them. Velodyne uses `time`, Ouster `t`, Hesai and RoboSense `timestamp`, and
/// Livox `offset_time`.
const char *kTimeFieldNames[] = {"time", "t", "timestamp", "time_stamp", "offset_time"};
const char *kRingFieldNames[] = {"ring", "line", "channel"};

/// Any stamp beyond this is a wall-clock time rather than an offset into the scan.
constexpr double kAbsoluteTimeThreshold = 1e6;

const PointField *findField(const PointCloud2 &msg, const std::string &name) {
    for (const auto &field : msg.fields) {
        if (field.name == name) return &field;
    }
    return nullptr;
}

const PointField *findAnyField(const PointCloud2 &msg, const char *const *names,
                               const std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (const auto *field = findField(msg, names[i])) return field;
    }
    return nullptr;
}

std::size_t scalarSize(const PointField &field) {
    switch (field.datatype) {
        case PointField::INT8: case PointField::UINT8: return 1;
        case PointField::INT16: case PointField::UINT16: return 2;
        case PointField::INT32: case PointField::UINT32: case PointField::FLOAT32: return 4;
        case PointField::FLOAT64: return 8;
        default: return 0;
    }
}

template <typename T>
double readNumber(const std::uint8_t *at, bool bigendian) {
    T value;
    std::uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, at, sizeof(T));
    const std::uint16_t one = 1;
    const bool host_bigendian = *reinterpret_cast<const std::uint8_t *>(&one) == 0;
    if (bigendian != host_bigendian) std::reverse(bytes, bytes + sizeof(T));
    std::memcpy(&value, bytes, sizeof(T));
    return value;
}

/// Layout and field bounds are checked once before reading any points.
double readScalar(const std::uint8_t *point, const PointField &field, bool bigendian) {
    const auto *at = point + field.offset;
    switch (field.datatype) {
        case PointField::INT8: return readNumber<std::int8_t>(at, bigendian);
        case PointField::UINT8: return readNumber<std::uint8_t>(at, bigendian);
        case PointField::INT16: return readNumber<std::int16_t>(at, bigendian);
        case PointField::UINT16: return readNumber<std::uint16_t>(at, bigendian);
        case PointField::INT32: return readNumber<std::int32_t>(at, bigendian);
        case PointField::UINT32: return readNumber<std::uint32_t>(at, bigendian);
        case PointField::FLOAT32: return readNumber<float>(at, bigendian);
        case PointField::FLOAT64: return readNumber<double>(at, bigendian);
        default: return std::numeric_limits<double>::quiet_NaN();
    }
}

/// Scale that turns a raw timing value into seconds. Integer timing fields count
/// nanoseconds; floating-point ones are already in seconds.
double timeScale(const PointField &field) {
    switch (field.datatype) {
        case PointField::UINT32:
        case PointField::INT32:
        case PointField::UINT16:
        case PointField::INT16:
            return 1e-9;
        default:
            return 1.0;
    }
}

}  // namespace

ConvertedScan toRawScan(const PointCloud2 &msg, const LidarType lidar_type) {
    ConvertedScan result;

    const auto *x_field = findField(msg, "x");
    const auto *y_field = findField(msg, "y");
    const auto *z_field = findField(msg, "z");
    if (!x_field || !y_field || !z_field) {
        result.warning = "point cloud has no x/y/z fields";
        return result;
    }

    const auto *intensity_field = findField(msg, "intensity");
    if (!intensity_field) intensity_field = findField(msg, "reflectivity");
    const auto *time_field =
        findAnyField(msg, kTimeFieldNames, sizeof(kTimeFieldNames) / sizeof(char *));
    const auto *ring_field =
        findAnyField(msg, kRingFieldNames, sizeof(kRingFieldNames) / sizeof(char *));

    if (msg.width == 0 || msg.height == 0) return result;
    const std::uint64_t row_bytes = std::uint64_t(msg.width) * msg.point_step;
    const std::uint64_t required = std::uint64_t(msg.height - 1) * msg.row_step + row_bytes;
    if (msg.point_step == 0 || msg.row_step < row_bytes || required > msg.data.size()) {
        result.warning = "invalid point cloud stride or truncated data";
        return result;
    }
    for (const auto *field : {x_field, y_field, z_field, intensity_field, time_field, ring_field}) {
        if (!field) continue;
        const auto size = scalarSize(*field);
        if (size == 0 || field->count == 0 || std::uint64_t(field->offset) + size > msg.point_step) {
            result.warning = "invalid point cloud field: " + field->name;
            return result;
        }
    }

    const std::size_t count = static_cast<std::size_t>(msg.width) * msg.height;
    result.scan.points.reserve(count);
    if (ring_field) result.scan.rings.reserve(count);

    const double scale = time_field ? timeScale(*time_field) : 1.0;
    const double header_time = toSeconds(msg.header.stamp);
    const bool timestamp_field = time_field &&
        (time_field->name == "timestamp" || time_field->name == "time_stamp");
    // The original Livox PointCloud2 schema stores absolute nanoseconds in a double.
    const bool livox_absolute_ns = lidar_type == LidarType::LivoxPcl && timestamp_field;
    const bool absolute_seconds = timestamp_field &&
        (lidar_type == LidarType::Hesai || lidar_type == LidarType::Robosense);
    double min_time = std::numeric_limits<double>::max();
    double max_time = std::numeric_limits<double>::lowest();

    for (std::size_t i = 0; i < count; ++i) {
        const std::uint8_t *point = msg.data.data() + (i / msg.width) * msg.row_step + (i % msg.width) * msg.point_step;

        Point out;
        out.position = Vec3(readScalar(point, *x_field, msg.is_bigendian), readScalar(point, *y_field, msg.is_bigendian),
                            readScalar(point, *z_field, msg.is_bigendian));
        if (!out.position.allFinite()) continue;
        if (intensity_field) out.intensity = static_cast<float>(readScalar(point, *intensity_field, msg.is_bigendian));

        if (time_field) {
            const double raw_stamp = readScalar(point, *time_field, msg.is_bigendian);
            double stamp = raw_stamp * scale;
            if (livox_absolute_ns) {
                stamp = (raw_stamp - header_time * 1e9) * 1e-9;
            } else if (absolute_seconds || std::abs(stamp) > kAbsoluteTimeThreshold) {
                // Keep the first-point/header offset: ScanBuffer anchors to the header.
                stamp -= header_time;
            }
            if (!std::isfinite(stamp) || std::abs(stamp) > std::numeric_limits<float>::max()) continue;
            out.time_offset = static_cast<float>(stamp);
            min_time = std::min(min_time, stamp);
            max_time = std::max(max_time, stamp);
        }

        result.scan.points.push_back(out);
        if (ring_field) {
            const double ring = readScalar(point, *ring_field, msg.is_bigendian);
            result.scan.rings.push_back(std::isfinite(ring) && ring >= 0 && ring < 65535
                ? static_cast<std::uint16_t>(ring) : std::numeric_limits<std::uint16_t>::max());
        }
    }

    if (!time_field) {
        result.scan.has_time_offsets = false;
        result.warning =
            "no per-point timing field; offsets will be reconstructed from azimuth, which "
            "needs a correct scan_rate and scan_line";
    } else if (max_time <= min_time) {
        // Every point carries the same stamp, which is no better than none.
        result.scan.has_time_offsets = false;
        result.warning = "per-point timing field is constant; falling back to azimuth";
    } else {
        result.duration = max_time - min_time;
        result.scan.time_min = min_time;
        result.scan.time_max = max_time;
    }

    if (!ring_field && !result.scan.has_time_offsets) {
        result.warning += "; no ring field either, so timing cannot be recovered";
    }

    return result;
}

}  // namespace ros_wrapper
}  // namespace genz_lio
