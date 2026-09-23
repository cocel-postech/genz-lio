#include "core/ScanBuffer.hpp"
#include "Check.hpp"
#include <set>
using namespace genz_lio;

RawScan sweep(float first, float last) {
    RawScan s;
    for (float t : {first, last}) {
        Point p; p.position = Vec3(5, 1, 1); p.time_offset = t; s.points.push_back(p);
    }
    return s;
}
void rewinds(bool imu_first) {
    PreprocessConfig config; config.lidar_type = LidarType::Ouster;
    ScanBuffer b(config);
    b.push(sweep(0, .3f), 100, .3);
    ImuSample m; m.timestamp = 100.1; b.push(m);
    ScanBuffer::BufferedScan scan; std::vector<ImuSample> imu;
    if (imu_first) {
        m.timestamp = 10.01;
        GENZ_CHECK(b.push(m) == ScanBuffer::Event::ImuStampsWentBackwards);
        GENZ_CHECK(b.pendingScans() == 0);
        GENZ_CHECK(b.push(sweep(0, .01f), 10, .01) == ScanBuffer::Event::None);
    } else {
        GENZ_CHECK(b.push(sweep(0, .01f), 10, .01) == ScanBuffer::Event::ScanStampsWentBackwards);
        GENZ_CHECK(!b.pop(scan, imu));  // The old 100 s IMU cannot complete a 10 s scan.
        m.timestamp = 10.01;
        GENZ_CHECK(b.push(m) == ScanBuffer::Event::None);
    }
    GENZ_CHECK(!b.pop(scan, imu));
    m.timestamp = 10.11; b.push(m);
    GENZ_CHECK(b.pop(scan, imu));
    GENZ_CHECK(scan.epoch == 1 && std::abs(scan.end_time - 10.1) < 1e-9);
    GENZ_CHECK(imu.size() == 1 && imu.front().timestamp == 10.01);
}
void hashAxes() {
    VoxelHash hash;
    for (int axis = 0; axis < 3; ++axis) {
        std::set<std::size_t> hashes;
        for (int i = -100; i <= 100; ++i) {
            VoxelKey key(1, 2, 3);
            if (axis == 0) key.x = i;
            if (axis == 1) key.y = i;
            if (axis == 2) key.z = i;
            hashes.insert(hash(key));
        }
        GENZ_CHECK(hashes.size() > 195);
    }
    GENZ_CHECK(hash(VoxelKey(1, 2, 3)) != hash(VoxelKey(1, 2, 3 + (1LL << 32))));
    GENZ_CHECK(hash(VoxelKey(1, 2, 3)) != hash(VoxelKey(1, 2 + (1LL << 32), 3)));
}
int main() { rewinds(false); rewinds(true); hashAxes(); }
