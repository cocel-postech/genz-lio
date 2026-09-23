// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "ScaleAwareVoxelizer.hpp"

#include "Downsample.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace genz_lio {
namespace {

/// Guards the normalizations below, which all divide by a setpoint-derived term.
constexpr double kEpsilon = 1e-9;

/// Maps a quantity to [0, 1] by saturating it at `limit`.
double saturatingRatio(const double value, const double limit) {
    if (limit <= kEpsilon) return 0.0;
    return std::min(std::abs(value), limit) / limit;
}

}  // namespace

ScaleAwareVoxelizer::ScaleAwareVoxelizer(const AdaptiveVoxelizationConfig &config,
                                         const double fixed_leaf_size)
    : config_(config),
      fixed_leaf_size_(fixed_leaf_size),
      leaf_size_(std::clamp(fixed_leaf_size, kMinLeafSize, kMaxLeafSize)) {}

double ScaleAwareVoxelizer::updateScaleIndicator(const PointCloud &probe) {
    double median_range = 0.0;
    if (!probe.empty()) {
        std::vector<double> squared_ranges;
        squared_ranges.reserve(probe.size());
        for (const auto &point : probe) {
            squared_ranges.push_back(point.position.squaredNorm());
        }
        const auto middle = squared_ranges.begin() + squared_ranges.size() / 2;
        std::nth_element(squared_ranges.begin(), middle, squared_ranges.end());
        median_range = std::sqrt(*middle);
    }

    // Sliding-window mean of the per-scan medians. Smoothing here keeps a single
    // cluttered scan from yanking the setpoint.
    last_median_range_ = median_range;
    median_window_.push_back(median_range);
    median_window_sum_ += median_range;
    if (static_cast<int>(median_window_.size()) > config_.window_size) {
        median_window_sum_ -= median_window_.front();
        median_window_.pop_front();
    }
    return median_window_sum_ / static_cast<double>(median_window_.size());
}

double ScaleAwareVoxelizer::computeSetpoint(const double scale_indicator) const {
    const double n_min = static_cast<double>(config_.min_points);
    const double n_max = static_cast<double>(config_.max_points);
    if (scale_indicator >= config_.scale_threshold) return n_max;

    // rho(t) = 1 - (1 - t)^p, which reaches n_max with zero slope when p > 1 and
    // so joins the saturated region smoothly.
    const double t = scale_indicator / config_.scale_threshold;
    const double rho = 1.0 - std::pow(1.0 - t, config_.setpoint_exponent);
    return n_min + (n_max - n_min) * rho;
}

void ScaleAwareVoxelizer::updateLeafSize(const double setpoint, const std::size_t point_count,
                                         const double scale_indicator, const double dt) {
    const double error = static_cast<double>(point_count) - setpoint;
    const double error_rate = (error - previous_error_) / dt;

    // Gain scheduling: each factor is normalized to [0, 1], then combined as a
    // geometric mean so that a low scale or a small error damps the response.
    const double error_ratio = saturatingRatio(error, setpoint * config_.error_sensitivity);
    const double error_rate_ratio =
        saturatingRatio(error_rate, setpoint * config_.error_rate_sensitivity / dt);
    const double scale_ratio =
        saturatingRatio(scale_indicator, config_.scale_threshold);

    const double p_gain = config_.p_gain_min + (config_.p_gain_max - config_.p_gain_min) *
                                                   std::sqrt(scale_ratio * error_ratio);
    const double d_gain = config_.d_gain_min + (config_.d_gain_max - config_.d_gain_min) *
                                                   std::sqrt(scale_ratio * error_rate_ratio);

    leaf_size_ = std::clamp(leaf_size_ + p_gain * error + d_gain * error_rate, kMinLeafSize,
                            kMaxLeafSize);
    previous_error_ = error;
    tracking_error_ = error;
}

ScaleAwareVoxelizer::Output ScaleAwareVoxelizer::process(const PointCloud &undistorted,
                                                         const double scan_time) {
    Output output;

    if (!config_.enable) {
        output.mapping = voxelDownsample(undistorted, fixed_leaf_size_);
        sortByTime(output.mapping);
        output.estimation = output.mapping;
        output.leaf_size = fixed_leaf_size_;
        return output;
    }

    // Probe the scan at the leaf size carried over from the previous scan; both
    // the scale indicator and the tracking error are read off this cloud.
    const PointCloud probe = voxelDownsample(undistorted, leaf_size_);
    const double scale_indicator = updateScaleIndicator(probe);
    const double setpoint = computeSetpoint(scale_indicator);

    const double dt =
        (previous_scan_time_ > 0.0) ? (scan_time - previous_scan_time_) : kDefaultScanPeriod;
    previous_scan_time_ = scan_time;
    updateLeafSize(setpoint, probe.size(), scale_indicator, dt > kEpsilon ? dt : kDefaultScanPeriod);

    // Re-voxelize at the corrected resolution. The estimation cloud is thinned
    // out of the mapping cloud rather than the raw scan, so its points are a
    // subset of what the map receives.
    output.mapping = voxelDownsample(undistorted, leaf_size_ * 0.5);
    output.estimation = voxelDownsample(output.mapping, leaf_size_);
    sortByTime(output.mapping);
    sortByTime(output.estimation);

    output.leaf_size = leaf_size_;
    output.median_range = lastMedianRange();
    output.scale_indicator = scale_indicator;
    output.setpoint = setpoint;
    output.tracking_error = tracking_error_;
    return output;
}

}  // namespace genz_lio
