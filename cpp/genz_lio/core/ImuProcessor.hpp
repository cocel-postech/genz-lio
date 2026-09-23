// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// IMU initialization, forward propagation of the ESIKF across a scan, and
// backward propagation to deskew that scan (paper Sec. V-A).
#pragma once

#include "Config.hpp"
#include "StateIkfom.hpp"
#include "Types.hpp"

#include <vector>

namespace genz_lio {

class ImuProcessor {
public:
    /// Scans consumed before the gravity direction, gyro bias and noise
    /// statistics are considered settled.
    static constexpr int kInitScanCount = 20;
    static constexpr double kGravity = 9.81;

    ImuProcessor();

    /// Clears stream history and initialization, preserving noise and extrinsics.
    void reset();

    void setExtrinsic(const Vec3 &translation, const Mat3 &rotation);
    void setGyrCov(const Vec3 &cov) { cov_gyr_scale_ = cov; }
    void setAccCov(const Vec3 &cov) { cov_acc_scale_ = cov; }
    void setGyrBiasCov(const Vec3 &cov) { cov_bias_gyr_ = cov; }
    void setAccBiasCov(const Vec3 &cov) { cov_bias_acc_ = cov; }

    /// Propagates `kf` across the bundle and writes the deskewed scan to
    /// `deskewed`. While the IMU is still initializing, `deskewed` is left
    /// untouched and `initialized()` stays false.
    void process(const MeasurementBundle &bundle, EsekfState &kf, PointCloud &deskewed);

    bool initialized() const { return !imu_need_init_; }

    /// Rotation that takes the odometry frame to a gravity-aligned world frame,
    /// estimated once from the mean acceleration during initialization.
    const Mat3 &initialGravityAlignment() const { return initial_r_wrt_g_; }

    double firstScanTime() const { return first_scan_time_; }

private:
    /// One propagated IMU state, kept so the scan can be deskewed backwards.
    struct ImuPose {
        double offset_time = 0.0;  ///< seconds from the start of the scan
        Vec3 acc = Vec3::Zero();   ///< acceleration in the world frame
        Vec3 gyr = Vec3::Zero();   ///< bias-corrected angular velocity
        Vec3 vel = Vec3::Zero();
        Vec3 pos = Vec3::Zero();
        Mat3 rot = Mat3::Identity();
    };

    void initializeImu(const MeasurementBundle &bundle, EsekfState &kf);
    void undistort(const MeasurementBundle &bundle, EsekfState &kf, PointCloud &cloud);

    Eigen::Matrix<double, 12, 12> process_noise_;

    Vec3 cov_acc_;
    Vec3 cov_gyr_;
    Vec3 cov_acc_scale_ = Vec3::Zero();
    Vec3 cov_gyr_scale_ = Vec3::Zero();
    Vec3 cov_bias_gyr_ = Vec3::Constant(0.0001);
    Vec3 cov_bias_acc_ = Vec3::Constant(0.0001);

    Vec3 mean_acc_;
    Vec3 mean_gyr_;
    Vec3 angvel_last_ = Vec3::Zero();
    Vec3 acc_s_last_ = Vec3::Zero();

    Mat3 lidar_r_wrt_imu_ = Mat3::Identity();
    Vec3 lidar_t_wrt_imu_ = Vec3::Zero();
    Mat3 initial_r_wrt_g_ = Mat3::Identity();

    std::vector<ImuPose> imu_poses_;
    ImuSample last_imu_;

    double first_scan_time_ = 0.0;
    double last_scan_end_time_ = 0.0;
    int init_iter_num_ = 1;
    bool first_frame_ = true;
    bool imu_need_init_ = true;
};

}  // namespace genz_lio
