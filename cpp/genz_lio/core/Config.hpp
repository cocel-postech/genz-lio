// This file is part of GenZ-LIO, released under the GNU GPL v2.
#pragma once

#include "Types.hpp"

#include <vector>

namespace genz_lio {

enum class LidarType {
    Livox = 1,   ///< Livox CustomMsg
    Velodyne,    ///< Velodyne-style PointCloud2 (per-point `time`)
    Ouster,      ///< Ouster-style PointCloud2 (per-point `t`)
    LivoxPcl,    ///< Livox published as PointCloud2
    Hesai,       ///< Hesai/Pandar-style PointCloud2 (absolute `timestamp`)
    Robosense,   ///< Robosense-style PointCloud2 (absolute `timestamp`)
};

struct PreprocessConfig {
    LidarType lidar_type = LidarType::Velodyne;
    int scan_line = 16;
    int scan_rate = 10;          ///< [Hz], only needed for spinning LiDARs
    double blind_min = 0.5;      ///< [m] drop returns closer than this
    double blind_max = 100.0;    ///< [m] drop returns farther than this
    int point_filter_num = 1;    ///< keep every n-th point
};

struct MappingConfig {
    int max_iteration = 4;       ///< ESIKF iterations per scan
    double down_sample_size = 0.25;  ///< [m] initial voxel-grid leaf size

    /// Root voxel edge length d_root [m]. Sets the octree root cell and feeds
    /// the discretization variance of the point-to-point covariance.
    double voxel_size = 1.0;
    int max_layer = 4;
    std::vector<int> layer_point_size = {5, 5, 5, 5, 5};
    double planar_threshold = 0.001;  ///< [m^2] upper bound on the smallest covariance eigenvalue
    double sigma_num = 3.0;          ///< point-to-plane residual gate
    int max_points_size = 100;
    int max_mature_points_size = 100;

    /// [m] discard map voxels farther than this from the sensor; <= 0 keeps all.
    double map_range = -1.0;

    bool extrinsic_est_en = false;
    Vec3 extrinsic_t = Vec3::Zero();
    Mat3 extrinsic_r = Mat3::Identity();
};

/// Scale-aware adaptive voxelization (paper Sec. IV). The leaf size is driven by
/// a PD controller whose setpoint follows the estimated spatial scale.
struct AdaptiveVoxelizationConfig {
    bool enable = true;

    int window_size = 5;                ///< N_w, median smoothing window
    double scale_threshold = 30.0;      ///< tau_m [m], scale at which the setpoint saturates
    int setpoint_exponent = 2;          ///< p, exponent of the saturating power map
    int min_points = 1000;              ///< N_min
    int max_points = 4000;              ///< N_max

    double error_sensitivity = 0.1;     ///< lambda_p, normalizes the tracking error
    double error_rate_sensitivity = 0.2;///< lambda_d, normalizes its derivative

    double p_gain_min = 5e-6;           ///< K_p,min
    double p_gain_max = 5e-5;           ///< K_p,max
    double d_gain_min = 5e-8;           ///< K_d,min
    double d_gain_max = 5e-7;           ///< K_d,max
};

/// Threshold on the point-to-point residual. The gate is scaled by each point's
/// own range, so a distant return is judged against a looser bound than a near
/// one, and widened by the motion the filter has had to correct so far.
struct AdaptiveThresholdConfig {
    /// Startup uncertainty and minimum sigma once pose corrections are available.
    double initial_threshold = 0.5;
    double max_range_motion = 100.0;
};

/// Hybrid-metric state update (paper Sec. V-E): point-to-plane and point-to-point
/// residuals fused through their respective covariances.
struct HybridMetricConfig {
    bool enable = true;

    /// lambda_po in eq. (30): R_comb = lambda_po * (R_norm + R_disc). Balances the
    /// numerical scale of the point-to-point covariance, which comes from a single
    /// correspondence, against the point-to-plane one, fitted from many points.
    double lambda_po = 0.05;

    double sigma_num = 3.0;   ///< point-to-point residual gate

    AdaptiveThresholdConfig adaptive_threshold;

    /// How many points a root voxel keeps as point-to-point search candidates.
    int max_points_per_voxel = 128;
    /// Each time a voxel subdivides, the child's budget is the parent's divided
    /// by this, so deeper cells hold proportionally fewer candidates.
    int reduction_ratio = 4;
};

struct NoiseModelConfig {
    double ranging_cov = 0.04;
    double angle_cov = 0.1;
    double acc_cov = 0.2;
    double gyr_cov = 0.05;
    double b_acc_cov = 0.0043;
    double b_gyr_cov = 0.000266;
};

struct Config {
    /// OpenMP worker limit for correspondence search and map updates. Zero uses
    /// detected physical cores, falling back to omp_get_num_procs() if topology
    /// is unavailable. A positive value sets an explicit limit. No CPU affinity
    /// is applied; this limit is set explicitly rather than using OMP_NUM_THREADS.
    int max_threads = 0;

    PreprocessConfig preprocess;
    MappingConfig mapping;
    AdaptiveVoxelizationConfig adaptive_voxelization;
    HybridMetricConfig hybrid_metric;
    NoiseModelConfig noise_model;
};

}  // namespace genz_lio
