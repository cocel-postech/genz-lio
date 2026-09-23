// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Scan preprocessing: range gating, decimation, and the per-point timing that
// deskewing depends on.
//
// Only the extraction of fields from a driver message is sensor-specific enough
// to belong in the ROS wrappers; everything that changes which points survive,
// or when they were measured, lives here so that every front end behaves alike.
#pragma once

#include "Config.hpp"
#include "Types.hpp"

#include <cstdint>
#include <vector>

namespace genz_lio {

/// One scan as handed over by a driver: positions and intensities, the per-point
/// times exactly as the driver reported them, and — when the sensor provides
/// them — the ring index of each return.
///
/// Drivers disagree on what those times are relative to. Velodyne on some
/// platforms stamps the message at the end of the sweep and reports negative
/// offsets back to its start; others stamp the start and count forwards. Both
/// are normalized in `Preprocessor::process`, which rebases every offset onto
/// the first return, so the rest of the pipeline only ever sees offsets in
/// [0, duration].
struct RawScan {
    PointCloud points;
    std::vector<std::uint16_t> rings;
    /// False when the driver gave no usable per-point timing, in which case the
    /// offsets are reconstructed from the azimuth of each return.
    bool has_time_offsets = true;
    /// Smallest and largest reported time, before rebasing.
    double time_min = 0.0;
    double time_max = 0.0;
    /// Timing has been validated/reconstructed before IMU synchronization.
    bool timing_prepared = false;
};

class Preprocessor {
public:
    explicit Preprocessor(const PreprocessConfig &config) : config_(config) {}

    /// Returns the points that survive gating and decimation, with their time
    /// offsets filled in.
    PointCloud process(const RawScan &raw) const;

    /// Remove invalid timing and derive bounds before choosing the IMU interval.
    /// Does not apply range gating or decimation. False means no usable timing.
    bool prepareTiming(RawScan &scan) const;

    const PreprocessConfig &config() const { return config_; }

private:
    /// Rebuilds per-point time offsets from the azimuth swept within each ring,
    /// for sensors that report no timing of their own.
    bool reconstructTimeOffsets(RawScan &scan) const;

    PreprocessConfig config_;
};

}  // namespace genz_lio
