#include "CloudConversion.hpp"
#include "ScanBuffer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using namespace genz_lio;
using namespace genz_lio::ros_wrapper;
namespace {
int failures = 0;
void check(bool ok, const char *name) {
    std::cout << (ok ? "PASS " : "FAIL ") << name << '\n';
    failures += !ok;
}
PointCloud2 message(const char *name = "time") {
    PointCloud2 m;
    m.header.stamp = toRosTime(1700000000.0);
    m.width = 3; m.height = 1; m.point_step = 20; m.row_step = 60;
    m.data.resize(60);
    for (int i = 0; i < 4; ++i) {
        PointField f;
        f.name = i == 0 ? "x" : i == 1 ? "y" : i == 2 ? "z" : name;
        f.offset = 4 * i; f.count = 1;
        f.datatype = i == 3 ? PointField::FLOAT64 : PointField::FLOAT32;
        m.fields.push_back(f);
    }
    for (int i = 0; i < 3; ++i) {
        float xyz[3] = {5, 1, 1};
        std::memcpy(m.data.data() + 20 * i, xyz, 12);
    }
    return m;
}
void times(PointCloud2 &m, std::initializer_list<double> values) {
    std::size_t i = 0;
    for (double t : values) { std::memcpy(m.data.data() + 20 * i + 12, &t, 8); ++i; }
}
void conversion() {
    auto m = message("timestamp");
    const double base = toSeconds(m.header.stamp);
    times(m, {base + .025, base + .050, base + .1});
    auto r = toRawScan(m, LidarType::Hesai);
    check(std::abs(r.scan.time_min - .025) < 1e-6, "absolute stamp retains header offset");
    // Absolute timestamps remain absolute even in bags whose epoch is near zero.
    m.header.stamp = toRosTime(10.0);
    times(m, {10.025, 10.050, 10.1});
    r = toRawScan(m, LidarType::Hesai);
    check(std::abs(r.scan.time_min - .025) < 1e-6, "Hesai simulated clock anchor");

    m.header.stamp = toRosTime(base);
    times(m, {base * 1e9, base * 1e9 + 5e7, base * 1e9 + 1e8});
    r = toRawScan(m, LidarType::LivoxPcl);
    check(std::abs(r.duration - .1) < 1e-6, "Livox absolute nanoseconds");
    m = message(); times(m, {-.1, -.05, .001});
    r = toRawScan(m, LidarType::Velodyne);
    check(std::abs(r.scan.time_min + .1) < 1e-7 && std::abs(r.duration - .101) < 1e-7,
          "negative relative seconds unchanged");

    m.width = 1; m.height = 2; m.row_step = 40; m.data.assign(80, 0);
    float a[3] = {1, 2, 3}, b[3] = {4, 5, 6};
    std::memcpy(m.data.data(), a, 12); std::memcpy(m.data.data() + 40, b, 12);
    double t = .1; std::memcpy(m.data.data() + 52, &t, 8);
    r = toRawScan(m, LidarType::Ouster);
    check(r.scan.points.size() == 2 && (r.scan.points[1].position - Vec3(4,5,6)).norm() < 1e-12,
          "organized rows with padding");
    for (int row = 0; row < 2; ++row) {
        for (const auto &f : m.fields) {
            const int n = f.datatype == PointField::FLOAT64 ? 8 : 4;
            auto begin = m.data.begin() + row * 40 + f.offset;
            std::reverse(begin, begin + n);
        }
    }
    m.is_bigendian = true;
    r = toRawScan(m, LidarType::Ouster);
    check(r.scan.points.size() == 2 && (r.scan.points[1].position - Vec3(4,5,6)).norm() < 1e-12
              && std::abs(r.duration - .1) < 1e-7, "big endian coordinates and time");
}
void pairing() {
    auto m = message(); times(m, {-.1, -.01, -1.0});
    auto r = toRawScan(m, LidarType::Velodyne);
    ScanBuffer buffer;
    buffer.push(std::move(r.scan), 10.0, r.duration);
    ImuSample sample; sample.timestamp = 9.95; buffer.push(sample);
    sample.timestamp = 10.0; buffer.push(sample);
    ScanBuffer::BufferedScan scan; std::vector<ImuSample> imu;
    check(buffer.pop(scan, imu), "valid sweep becomes ready");
    const auto points = Preprocessor(PreprocessConfig{}).process(scan.scan);
    check(std::abs(scan.begin_time - 9.9) < 1e-6 && std::abs(scan.end_time - 9.99) < 1e-6,
          "rejected timestamp cannot move IMU interval");
    check(points.size() == 2 && std::abs(points.front().time_offset) < 1e-6 &&
              points.back().time_offset < .1, "deskew origin agrees with paired interval");
    check(imu.size() == 1 && std::abs(imu.front().timestamp - 9.95) < 1e-9,
          "IMU after sweep retained for next scan");
}

void reconstructionAndInvalidInput() {
    PreprocessConfig config;
    config.scan_rate = 5; config.scan_line = 1;
    RawScan raw; raw.has_time_offsets = false;
    for (double yaw : {0.0, -1.0, -2.0, -3.0, 2.0, 1.0, .1}) {
        Point p; p.position = Vec3(5 * std::cos(yaw), 5 * std::sin(yaw), 1);
        raw.points.push_back(p); raw.rings.push_back(0);
    }
    ScanBuffer buffer(config);
    check(buffer.push(std::move(raw), 10, 0) == ScanBuffer::Event::None,
          "missing offsets reconstructed before synchronization");
    ImuSample sample; sample.timestamp = 10.11; buffer.push(sample);
    ScanBuffer::BufferedScan scan; std::vector<ImuSample> imu;
    check(!buffer.pop(scan, imu), "5 Hz sweep waits beyond old 100 ms fallback");
    sample.timestamp = 10.19; buffer.push(sample);
    sample.timestamp = 10.21; buffer.push(sample);
    check(buffer.pop(scan, imu), "reconstructed sweep ready at correct IMU boundary");
    const auto points = Preprocessor(config).process(scan.scan);
    check(points.size() == 7 && points.front().time_offset == 0 &&
              points.back().time_offset > .19 &&
              std::abs(scan.end_time - scan.begin_time - points.back().time_offset) < 1e-8,
          "reconstructed first point retained and duration matches deskew");
    check(imu.size() == 2 && imu.back().timestamp == 10.19, "5 Hz bundle includes late IMU");
    const auto before = scan.scan.points;
    check(Preprocessor(config).prepareTiming(scan.scan) &&
              before.size() == scan.scan.points.size() &&
              before.front().time_offset == scan.scan.points.front().time_offset,
          "preparing timing twice is idempotent");

    raw = RawScan{}; raw.has_time_offsets = false;
    Point p; p.position = Vec3(5, 1, 1); raw.points.push_back(p);
    check(buffer.push(std::move(raw), 11, 0) == ScanBuffer::Event::InvalidScan &&
              buffer.pendingScans() == 0, "unrecoverable timing rejected before pairing");

    auto m = message(); times(m, {-.1, std::numeric_limits<double>::quiet_NaN(), -.01});
    auto r = toRawScan(m, LidarType::Velodyne);
    check(r.scan.points.size() == 2 && std::isfinite(r.duration), "NaN timing cannot enter bounds");
    // A truncated payload and an out-of-bounds scalar used to be read unchecked.
    m.data.resize(15);
    r = toRawScan(m, LidarType::Velodyne);
    check(r.scan.points.empty() && !r.warning.empty(), "truncated payload rejected");
    m = message(); m.fields.back().offset = 19;
    r = toRawScan(m, LidarType::Velodyne);
    check(r.scan.points.empty() && !r.warning.empty(), "out of bounds field rejected");
    m = message(); m.row_step = 1;
    r = toRawScan(m, LidarType::Velodyne);
    check(r.scan.points.empty() && !r.warning.empty(), "invalid row stride rejected");

    // Livox CustomMsg uses the same buffer; packets need not be sorted by offset.
    config.lidar_type = LidarType::Livox;
    ScanBuffer livox(config);
    raw = RawScan{};
    for (float t : {.02f, .099f, .01f}) { p.time_offset = t; raw.points.push_back(p); }
    livox.push(std::move(raw), 20, .01);
    sample.timestamp = 20.11; livox.push(sample);
    check(livox.pop(scan, imu) && std::abs(scan.begin_time - 20.01) < 1e-7 &&
              std::abs(scan.end_time - 20.099) < 1e-7, "unsorted CustomMsg extrema include last firing");
}
}
int main() { conversion(); pairing(); reconstructionAndInvalidInput(); return failures ? 1 : 0; }
