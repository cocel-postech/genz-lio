// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Scale-aware adaptive voxelization (paper Sec. IV).
//
// A fixed downsampling resolution suits one spatial scale: too coarse in a
// stairwell, too fine in the open. This module estimates the scale of the
// current scan and drives the leaf size with a PD controller so that the number
// of surviving points tracks a scale-dependent setpoint. The controller gains
// are themselves scheduled on the estimated scale and on the tracking error, so
// that the leaf size responds quickly during confined-open transitions without
// overshooting.
#pragma once

#include "Config.hpp"
#include "Types.hpp"

#include <deque>

namespace genz_lio {

class ScaleAwareVoxelizer {
public:
    /// Bounds on the leaf size, [d_min, d_max] in the paper.
    static constexpr double kMinLeafSize = 0.02;
    static constexpr double kMaxLeafSize = 1.0;
    /// Assumed scan period for the very first scan, before two stamps exist.
    static constexpr double kDefaultScanPeriod = 0.1;

    struct Output {
        /// Map-insertion cloud: half the adaptive leaf size when enabled;
        /// otherwise the same fixed-resolution cloud used for estimation.
        PointCloud mapping;
        /// Coarser cloud that forms the residuals of the state update.
        PointCloud estimation;

        double leaf_size = 0.0;
        /// This scan's own median range, before the sliding window smooths it.
        double median_range = 0.0;
        double scale_indicator = 0.0;  ///< m̄_t, the smoothed median range
        double setpoint = 0.0;         ///< N_desired,t
        double tracking_error = 0.0;   ///< e_t, probe count minus setpoint
    };

    /// `fixed_leaf_size` is both the starting point for the controller and the
    /// resolution used outright when adaptive voxelization is disabled.
    ScaleAwareVoxelizer(const AdaptiveVoxelizationConfig &config, double fixed_leaf_size);

    /// Downsamples one undistorted scan. `scan_time` is the scan end stamp, used
    /// only to measure the interval between successive calls.
    Output process(const PointCloud &undistorted, double scan_time);

    double leafSize() const { return leaf_size_; }

private:
    /// Median range of the probe cloud, smoothed over a sliding window.
    double updateScaleIndicator(const PointCloud &probe);

    /// The median this scan contributed, before smoothing.
    double lastMedianRange() const { return last_median_range_; }

    /// N_desired,t: saturating power map from the scale indicator to a point count.
    double computeSetpoint(double scale_indicator) const;

    /// One PD step on the leaf size, with gains scheduled by scale and error.
    void updateLeafSize(double setpoint, std::size_t point_count, double scale_indicator,
                        double dt);

    AdaptiveVoxelizationConfig config_;
    double fixed_leaf_size_;
    double last_median_range_ = 0.0;
    double leaf_size_;

    std::deque<double> median_window_;
    double median_window_sum_ = 0.0;

    double previous_error_ = 0.0;
    double previous_scan_time_ = 0.0;
    double tracking_error_ = 0.0;
};

}  // namespace genz_lio
