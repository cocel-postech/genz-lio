// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "ImuProcessor.hpp"

#include "Downsample.hpp"
#include "SO3Math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace genz_lio {
namespace {

Vec3 rotationToYpr(const Mat3 &R) {
    const Vec3 n = R.col(0);
    const Vec3 o = R.col(1);
    const Vec3 a = R.col(2);

    const double y = std::atan2(n(1), n(0));
    const double p = std::atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
    const double r = std::atan2(a(0) * std::sin(y) - a(1) * std::cos(y),
                                -o(0) * std::sin(y) + o(1) * std::cos(y));
    return Vec3(y, p, r) / M_PI * 180.0;
}

Mat3 yprToRotation(const Vec3 &ypr) {
    const double y = ypr(0) / 180.0 * M_PI;
    const double p = ypr(1) / 180.0 * M_PI;
    const double r = ypr(2) / 180.0 * M_PI;

    Mat3 Rz;
    Rz << std::cos(y), -std::sin(y), 0, std::sin(y), std::cos(y), 0, 0, 0, 1;
    Mat3 Ry;
    Ry << std::cos(p), 0, std::sin(p), 0, 1, 0, -std::sin(p), 0, std::cos(p);
    Mat3 Rx;
    Rx << 1, 0, 0, 0, std::cos(r), -std::sin(r), 0, std::sin(r), std::cos(r);
    return Rz * Ry * Rx;
}

/// Rotation that brings the measured gravity onto +z, with the free yaw removed
/// so that the world frame is fixed rather than arbitrary about the vertical.
Mat3 gravityToRotation(const Vec3 &g) {
    Mat3 R0 = Eigen::Quaterniond::FromTwoVectors(g.normalized(), Vec3(0, 0, 1)).toRotationMatrix();
    const double yaw = rotationToYpr(R0).x();
    return yprToRotation(Vec3(-yaw, 0, 0)) * R0;
}

}  // namespace

ImuProcessor::ImuProcessor() {
    process_noise_ = processNoiseCov();
    reset();
}

void ImuProcessor::reset() {
    cov_acc_ = Vec3(0.1, 0.1, 0.1);
    cov_gyr_ = Vec3(0.1, 0.1, 0.1);
    process_noise_ = processNoiseCov();
    mean_acc_ = Vec3(0, 0, -1.0);
    mean_gyr_ = Vec3::Zero();
    angvel_last_ = Vec3::Zero();
    acc_s_last_ = Vec3::Zero();
    imu_poses_.clear();
    last_imu_ = ImuSample{};
    init_iter_num_ = 1;
    imu_need_init_ = true;
    first_frame_ = true;
    first_scan_time_ = 0.0;
    last_scan_end_time_ = 0.0;
    initial_r_wrt_g_ = Mat3::Identity();
}

void ImuProcessor::setExtrinsic(const Vec3 &translation, const Mat3 &rotation) {
    lidar_t_wrt_imu_ = translation;
    lidar_r_wrt_imu_ = rotation;
}

void ImuProcessor::initializeImu(const MeasurementBundle &bundle, EsekfState &kf) {
    if (first_frame_) {
        reset();
        init_iter_num_ = 1;
        first_frame_ = false;
        mean_acc_ = bundle.imu.front().linear_acceleration;
        mean_gyr_ = bundle.imu.front().angular_velocity;
        first_scan_time_ = bundle.scan_begin_time;
    }

    // Running mean and variance of the samples collected while stationary.
    for (const auto &imu : bundle.imu) {
        const Vec3 &acc = imu.linear_acceleration;
        const Vec3 &gyr = imu.angular_velocity;
        const double n = static_cast<double>(init_iter_num_);

        mean_acc_ += (acc - mean_acc_) / n;
        mean_gyr_ += (gyr - mean_gyr_) / n;
        cov_acc_ = cov_acc_ * (n - 1.0) / n +
                   (acc - mean_acc_).cwiseProduct(acc - mean_acc_) * (n - 1.0) / (n * n);
        cov_gyr_ = cov_gyr_ * (n - 1.0) / n +
                   (gyr - mean_gyr_).cwiseProduct(gyr - mean_gyr_) * (n - 1.0) / (n * n);
        ++init_iter_num_;
    }

    state_ikfom init_state = kf.get_x();
    init_state.grav = S2(-mean_acc_ / mean_acc_.norm() * kGravity);
    init_state.bg = mean_gyr_;
    init_state.offset_T_L_I = lidar_t_wrt_imu_;
    init_state.offset_R_L_I = lidar_r_wrt_imu_;
    kf.change_x(init_state);

    initial_r_wrt_g_ = gravityToRotation(mean_acc_);

    EsekfState::cov init_P = kf.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf.change_P(init_P);

    last_imu_ = bundle.imu.back();
}

void ImuProcessor::undistort(const MeasurementBundle &bundle, EsekfState &kf, PointCloud &cloud) {
    // Bridge the gap to the previous scan by prepending its last IMU sample.
    std::vector<ImuSample> imu;
    imu.reserve(bundle.imu.size() + 1);
    imu.push_back(last_imu_);
    imu.insert(imu.end(), bundle.imu.begin(), bundle.imu.end());

    const double imu_end_time = imu.back().timestamp;
    const double scan_begin_time = bundle.scan_begin_time;
    const double scan_end_time = bundle.scan_end_time;

    cloud = bundle.scan;
    sortByTime(cloud);

    state_ikfom imu_state = kf.get_x();
    imu_poses_.clear();
    imu_poses_.push_back({0.0, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                          imu_state.rot.toRotationMatrix()});

    // Forward propagation: integrate the filter through every IMU interval and
    // record the pose at each sample.
    input_ikfom in;
    double dt = 0.0;
    for (std::size_t i = 0; i + 1 < imu.size(); ++i) {
        const ImuSample &head = imu[i];
        const ImuSample &tail = imu[i + 1];
        if (tail.timestamp < last_scan_end_time_) continue;

        const Vec3 angvel_avr = 0.5 * (head.angular_velocity + tail.angular_velocity);
        Vec3 acc_avr = 0.5 * (head.linear_acceleration + tail.linear_acceleration);
        // Rescale to m/s^2 using the gravity magnitude observed at initialization.
        acc_avr *= kGravity / mean_acc_.norm();

        dt = (head.timestamp < last_scan_end_time_) ? (tail.timestamp - last_scan_end_time_)
                                                    : (tail.timestamp - head.timestamp);

        in.acc = acc_avr;
        in.gyro = angvel_avr;
        process_noise_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        process_noise_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        process_noise_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        process_noise_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
        kf.predict(dt, process_noise_, in);

        imu_state = kf.get_x();
        angvel_last_ = angvel_avr - imu_state.bg;
        acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
        for (int axis = 0; axis < 3; ++axis) acc_s_last_[axis] += imu_state.grav[axis];

        imu_poses_.push_back({tail.timestamp - scan_begin_time, acc_s_last_, angvel_last_,
                              imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()});
    }

    // The bundle contains samples at or before scan end. Do not turn a small
    // floating-point overshoot into additional forward propagation.
    dt = std::max(0.0, scan_end_time - imu_end_time);
    kf.predict(dt, process_noise_, in);

    imu_state = kf.get_x();
    last_imu_ = bundle.imu.back();
    last_scan_end_time_ = scan_end_time;

    if (cloud.empty()) return;

    // Walk backwards, consuming each point exactly once. The last interval
    // includes offset zero; that return also needs the scan-end transform.
    std::size_t remaining = cloud.size();
    for (auto pose = imu_poses_.end() - 1; pose != imu_poses_.begin() && remaining; --pose) {
        const ImuPose &head = *(pose - 1);
        const ImuPose &tail = *pose;

        while (remaining && (cloud[remaining - 1].time_offset > head.offset_time ||
                             pose == imu_poses_.begin() + 1)) {
            Point *point = &cloud[--remaining];
            dt = point->time_offset - head.offset_time;

            const Mat3 R_i = head.rot * expSO3<double>(tail.gyr, dt);
            const Vec3 T_ei =
                head.pos + head.vel * dt + 0.5 * tail.acc * dt * dt - imu_state.pos;
            point->position =
                imu_state.offset_R_L_I.conjugate() *
                (imu_state.rot.conjugate() *
                     (R_i * (imu_state.offset_R_L_I * point->position + imu_state.offset_T_L_I) +
                      T_ei) -
                 imu_state.offset_T_L_I);

        }
    }
}

void ImuProcessor::process(const MeasurementBundle &bundle, EsekfState &kf, PointCloud &deskewed) {
    if (!std::isfinite(bundle.scan_begin_time) || !std::isfinite(bundle.scan_end_time) ||
        bundle.scan_end_time < bundle.scan_begin_time)
        throw std::invalid_argument("scan times must be finite and ordered");
    if (bundle.imu.empty()) return;
    for (std::size_t i = 0; i < bundle.imu.size(); ++i) {
        const auto &sample = bundle.imu[i];
        if (!std::isfinite(sample.timestamp) || !sample.linear_acceleration.allFinite() ||
            !sample.angular_velocity.allFinite())
            throw std::invalid_argument("IMU samples must be finite");
        if (i && sample.timestamp < bundle.imu[i - 1].timestamp)
            throw std::invalid_argument("IMU samples must be ordered");
    }
    // Stream synchronizers retain future samples for the next scan. Direct
    // C++/Python callers must honor the same contract (1 us rounding tolerance).
    if (bundle.imu.back().timestamp > bundle.scan_end_time + 1e-6)
        throw std::invalid_argument("IMU samples after scan end must be retained for the next scan");

    if (imu_need_init_) {
        initializeImu(bundle, kf);
        last_imu_ = bundle.imu.back();
        if (init_iter_num_ > kInitScanCount) {
            // The initial covariance was accumulated on raw accelerometer units;
            // rescale it before switching to the configured noise model.
            cov_acc_ *= std::pow(kGravity / mean_acc_.norm(), 2);
            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            imu_need_init_ = false;
        }
        return;
    }

    undistort(bundle, kf, deskewed);
}

}  // namespace genz_lio
