// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// The odometry pipeline: preprocessing, IMU propagation and deskewing,
// scale-aware adaptive voxelization, the hybrid-metric ESIKF update, and the
// map update that follows it.
//
// One instance owns all of its state, so several pipelines can run side by side
// in one process.
#pragma once

#include "Config.hpp"
#include "HybridMetricUpdate.hpp"
#include "ImuProcessor.hpp"
#include "Preprocessor.hpp"
#include "ScaleAwareVoxelizer.hpp"
#include "StateIkfom.hpp"
#include "Types.hpp"
#include "VoxelMap.hpp"

#include <memory>
#include <vector>

namespace genz_lio {

class GenZLIO {
public:
    struct Result {
        /// False while the IMU is still initializing, or when a scan yielded no
        /// usable correspondences.
        bool valid = false;
        double timestamp = 0.0;

        /// World frame to IMU body frame.
        Pose pose;
        Vec3 velocity = Vec3::Zero();

        /// The scan after deskewing, in the LiDAR frame.
        PointCloud deskewed;

        /// The actual adaptive voxelization output used by the filter, also in
        /// the LiDAR frame. Empty on the initial map-seeding scan.
        PointCloud voxelized;

        // Diagnostics, mirroring what the reference implementation printed.
        double leaf_size = 0.0;
        double median_range = 0.0;
        double scale_indicator = 0.0;
        double setpoint = 0.0;
        int scan_points = 0;
        int voxelized_points = 0;
        int mapping_points = 0;
        int plane_matches = 0;
        int point_matches = 0;
        double processing_time_ms = 0.0;

        /// Set when the ESIKF update produced a non-finite state and was rolled
        /// back to the IMU prediction. The scan is not folded into the map.
        bool update_rejected = false;

        /// Where the frame's time went, in milliseconds.
        struct Timing {
            double preprocess = 0.0;
            double deskew = 0.0;
            double voxelize = 0.0;
            double covariance = 0.0;
            double update = 0.0;
            double map = 0.0;
        } timing;
    };

    explicit GenZLIO(const Config &config);
    ~GenZLIO();

    GenZLIO(const GenZLIO &) = delete;
    GenZLIO &operator=(const GenZLIO &) = delete;

    /// Consumes one scan together with the IMU samples spanning it.
    Result registerScan(const RawScan &raw_scan, double scan_begin_time, double scan_end_time,
                        const std::vector<ImuSample> &imu);

    /// Discards the map and the filter state, keeping the configuration.
    void reset();

    bool initialized() const { return imu_processor_.initialized() && map_built_; }
    state_ikfom state() const { return kf_.get_x(); }
    EsekfState::cov covariance() const { return kf_.get_P(); }
    const VoxelMapType &map() const { return voxel_map_; }
    const Config &config() const { return config_; }

    /// Rotation from the odometry frame to a gravity-aligned world frame.
    const Mat3 &gravityAlignment() const { return imu_processor_.initialGravityAlignment(); }

    const std::vector<PointToPlaneMatch> &planeMatches() const { return update_.planeMatches(); }
    const std::vector<PointToPointMatch> &pointMatches() const { return update_.pointMatches(); }

    /// Optional, single-consumer display journal. No tracking work is done by
    /// headless/ROS callers. Keys describe completed map updates, never raw scans.
    struct MapChanges {
        bool reset = false;
        std::vector<VoxelKey> updated, removed;
    };
    void startMapTracking();
    void stopMapTracking();
    MapChanges takeMapChanges();

private:
    void clearMap();
    void seedMap(const PointCloud &cloud);
    void foldScanIntoMap(const PointCloud &cloud, const std::vector<Mat3> &body_covariances);
    std::vector<Mat3> bodyCovariances(const PointCloud &cloud) const;
    void installObservationModel();

    Config config_;
    int max_threads_ = 1;
    Preprocessor preprocessor_;
    ImuProcessor imu_processor_;
    ScaleAwareVoxelizer voxelizer_;
    HybridMetricUpdate update_;

    EsekfState kf_;
    VoxelMapType voxel_map_;
    bool map_built_ = false;
    bool track_map_ = false;
    bool display_map_reset_ = false;
    tsl::robin_map<VoxelKey, bool, VoxelHash> display_map_changes_;

    /// IKFoM holds a pointer to this array for the lifetime of the filter.
    std::array<double, 23> convergence_limit_;
};

}  // namespace genz_lio
