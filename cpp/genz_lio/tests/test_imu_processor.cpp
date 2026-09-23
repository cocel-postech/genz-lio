// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "core/ImuProcessor.hpp"
#include "Check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

using genz_lio::EsekfState;
using genz_lio::ImuProcessor;
using genz_lio::ImuSample;
using genz_lio::MeasurementBundle;
using genz_lio::PointCloud;
using genz_lio::Vec3;

/// A scan bundled with IMU samples held at a constant acceleration and rate.
MeasurementBundle makeBundle(const double begin, const double duration, const Vec3 &acc,
                             const Vec3 &gyr, const int point_count = 100) {
    MeasurementBundle bundle;
    bundle.scan_begin_time = begin;
    bundle.scan_end_time = begin + duration;

    for (int i = 0; i <= 20; ++i) {
        ImuSample sample;
        sample.timestamp = begin + duration * i / 20.0;
        sample.linear_acceleration = acc;
        sample.angular_velocity = gyr;
        bundle.imu.push_back(sample);
    }

    for (int i = 0; i < point_count; ++i) {
        genz_lio::Point p;
        const double t = static_cast<double>(i) / point_count;
        p.position = Vec3(5.0 + t, 1.0, 0.5);
        p.time_offset = static_cast<float>(t * duration);
        bundle.scan.push_back(p);
    }
    return bundle;
}

EsekfState makeFilter() {
    EsekfState kf;
    static double convergence_limit[23];
    std::fill(convergence_limit, convergence_limit + 23, 0.001);
    kf.init_dyn_share(
        genz_lio::getF, genz_lio::dfDx, genz_lio::dfDw,
        [](genz_lio::state_ikfom &, esekfom::dyn_share_datastruct<double> &ekfom_data) {
            ekfom_data.valid = false;  // measurement model is exercised elsewhere
        },
        1, convergence_limit);
    return kf;
}

void initializationTakesSeveralScans() {
    ImuProcessor imu;
    imu.setGyrCov(Vec3(0.05, 0.05, 0.05));
    imu.setAccCov(Vec3(0.2, 0.2, 0.2));
    EsekfState kf = makeFilter();

    GENZ_CHECK(!imu.initialized());

    PointCloud deskewed;
    const Vec3 gravity(0.0, 0.0, 9.81);
    for (int scan = 0; scan < 3; ++scan) {
        auto bundle = makeBundle(0.1 * scan, 0.1, gravity, Vec3::Zero());
        imu.process(bundle, kf, deskewed);
    }
    // 21 samples per scan clear the 20-sample threshold on the first bundle.
    GENZ_CHECK(imu.initialized());
    GENZ_CHECK(imu.firstScanTime() == 0.0);
}

void gravityAlignmentUprightIsIdentity() {
    ImuProcessor imu;
    EsekfState kf = makeFilter();
    PointCloud deskewed;
    auto bundle = makeBundle(0.0, 0.1, Vec3(0.0, 0.0, 9.81), Vec3::Zero());
    imu.process(bundle, kf, deskewed);

    const auto &R = imu.initialGravityAlignment();
    GENZ_CHECK((R - genz_lio::Mat3::Identity()).norm() < 1e-9);
}

void gravityAlignmentOnItsSideMapsAccelerationToZ() {
    ImuProcessor imu;
    EsekfState kf = makeFilter();
    PointCloud deskewed;
    const Vec3 acc(9.81, 0.0, 0.0);  // sensor lying on its side
    auto bundle = makeBundle(0.0, 0.1, acc, Vec3::Zero());
    imu.process(bundle, kf, deskewed);

    const Vec3 aligned = imu.initialGravityAlignment() * acc.normalized();
    GENZ_CHECK((aligned - Vec3(0.0, 0.0, 1.0)).norm() < 1e-9);
}

void deskewPreservesPointsAndStaysFinite() {
    ImuProcessor imu;
    imu.setGyrCov(Vec3(0.05, 0.05, 0.05));
    imu.setAccCov(Vec3(0.2, 0.2, 0.2));
    imu.setGyrBiasCov(Vec3(1e-4, 1e-4, 1e-4));
    imu.setAccBiasCov(Vec3(1e-3, 1e-3, 1e-3));
    EsekfState kf = makeFilter();

    PointCloud deskewed;
    const Vec3 gravity(0.0, 0.0, 9.81);
    auto init = makeBundle(0.0, 0.1, gravity, Vec3::Zero());
    imu.process(init, kf, deskewed);
    GENZ_CHECK(imu.initialized());

    // Now rotate about z while the scan is being swept.
    auto moving = makeBundle(0.1, 0.1, gravity, Vec3(0.0, 0.0, 0.5), 500);
    imu.process(moving, kf, deskewed);

    GENZ_CHECK(deskewed.size() == moving.scan.size());
    for (const auto &p : deskewed) {
        GENZ_CHECK(p.position.allFinite());
        GENZ_CHECK(p.position.norm() < 100.0);  // a 5 m return cannot fly away
    }
    // Deskewing leaves the scan ordered by time.
    for (std::size_t i = 1; i < deskewed.size(); ++i) {
        GENZ_CHECK(deskewed[i - 1].time_offset <= deskewed[i].time_offset);
    }
}

void emptyImuIsIgnored() {
    ImuProcessor imu;
    EsekfState kf = makeFilter();
    PointCloud deskewed;
    MeasurementBundle empty;
    imu.process(empty, kf, deskewed);
    GENZ_CHECK(!imu.initialized());
    GENZ_CHECK(deskewed.empty());
}

}  // namespace

int main() {
    initializationTakesSeveralScans();
    gravityAlignmentUprightIsIdentity();
    gravityAlignmentOnItsSideMapsAccelerationToZ();
    deskewPreservesPointsAndStaysFinite();
    emptyImuIsIgnored();
    std::printf("imu processor ok\n");
    return 0;
}
