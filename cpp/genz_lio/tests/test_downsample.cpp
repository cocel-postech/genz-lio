// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "core/Downsample.hpp"
#include "Check.hpp"

#include <cmath>
#include <cstdio>
#include <random>

namespace {

genz_lio::Point makePoint(const double x, const double y, const double z,
                          const float intensity = 0.0f, const float time = 0.0f) {
    genz_lio::Point p;
    p.position = genz_lio::Vec3(x, y, z);
    p.intensity = intensity;
    p.time_offset = time;
    return p;
}

void oneCentroidPerCell() {
    // Two points inside the cell [0,1)^3 and one inside [1,2)x[0,1)^2.
    const genz_lio::PointCloud cloud = {
        makePoint(0.1, 0.1, 0.1, 10.0f, 0.0f),
        makePoint(0.3, 0.3, 0.3, 20.0f, 0.2f),
        makePoint(1.5, 0.5, 0.5, 30.0f, 0.4f),
    };

    const auto out = genz_lio::voxelDownsample(cloud, 1.0);
    GENZ_CHECK(out.size() == 2);

    // Locate the merged cell and check it carries the centroid of its members.
    bool found_merged = false;
    for (const auto &p : out) {
        if (p.position.x() < 1.0) {
            found_merged = true;
            GENZ_CHECK((p.position - genz_lio::Vec3(0.2, 0.2, 0.2)).norm() < 1e-12);
            GENZ_CHECK(std::abs(p.intensity - 15.0f) < 1e-5f);
            GENZ_CHECK(std::abs(p.time_offset - 0.1f) < 1e-5f);
        }
    }
    GENZ_CHECK(found_merged);
}

void negativeCoordinatesFloorCorrectly() {
    // -0.5 and -0.1 both belong to cell -1; 0.5 belongs to cell 0.
    const genz_lio::PointCloud cloud = {
        makePoint(-0.5, 0.0, 0.0),
        makePoint(-0.1, 0.0, 0.0),
        makePoint(0.5, 0.0, 0.0),
    };
    GENZ_CHECK(genz_lio::voxelDownsample(cloud, 1.0).size() == 2);
}

void nonPositiveLeafIsPassThrough() {
    const genz_lio::PointCloud cloud = {makePoint(0.1, 0.1, 0.1), makePoint(0.2, 0.2, 0.2)};
    GENZ_CHECK(genz_lio::voxelDownsample(cloud, 0.0).size() == cloud.size());
    GENZ_CHECK(genz_lio::voxelDownsample(cloud, -1.0).size() == cloud.size());
    GENZ_CHECK(genz_lio::voxelDownsample({}, 0.5).empty());
}

void coarserLeafKeepsFewerPoints() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-20.0, 20.0);
    genz_lio::PointCloud cloud;
    cloud.reserve(20000);
    for (int i = 0; i < 20000; ++i) {
        cloud.push_back(makePoint(dist(rng), dist(rng), dist(rng)));
    }

    std::size_t previous = cloud.size();
    for (const double leaf : {0.1, 0.5, 1.0, 2.0, 5.0}) {
        const std::size_t current = genz_lio::voxelDownsample(cloud, leaf).size();
        GENZ_CHECK(current <= previous);
        previous = current;
    }
    GENZ_CHECK(previous < cloud.size());
}

void sortByTimeOrders() {
    genz_lio::PointCloud cloud = {makePoint(0, 0, 0, 0.0f, 0.3f), makePoint(1, 0, 0, 0.0f, 0.1f),
                                  makePoint(2, 0, 0, 0.0f, 0.2f)};
    genz_lio::sortByTime(cloud);
    GENZ_CHECK(cloud[0].time_offset < cloud[1].time_offset);
    GENZ_CHECK(cloud[1].time_offset < cloud[2].time_offset);
}

}  // namespace

int main() {
    oneCentroidPerCell();
    negativeCoordinatesFloorCorrectly();
    nonPositiveLeafIsPassThrough();
    coarserLeafKeepsFewerPoints();
    sortByTimeOrders();
    std::printf("downsample ok\n");
    return 0;
}
