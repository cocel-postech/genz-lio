// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Pairing the two streams. Scans and IMU samples arrive on separate callbacks
// and neither is complete when it does, so both are buffered until a scan's
// inertial span has fully landed. Deliberately free of any ROS type, so the
// ROS 1 and ROS 2 front ends share it unchanged.
#pragma once

#include "core/Preprocessor.hpp"
#include "core/Types.hpp"

#include <deque>
#include <mutex>
#include <limits>
#include <vector>

namespace genz_lio {

class ScanBuffer {
public:
    explicit ScanBuffer(const PreprocessConfig &config = {}) : preprocessor_(config) {}
    /// Set once during node initialization, before either stream is received.
    void setPreprocessConfig(const PreprocessConfig &config);
    struct BufferedScan {
        RawScan scan;
        double begin_time = 0.0;
        double end_time = 0.0;
        std::uint64_t epoch = 0;
    };

    /// Reason a push or pop did something worth telling the user about.
    enum class Event { None, ScanStampsWentBackwards, ImuStampsWentBackwards, InvalidScan, InvalidImu };

    /// `stamp` is the message stamp; where the sweep sits relative to it comes
    /// from validated/reconstructed point times. The legacy duration argument
    /// is ignored: unvalidated extrema must not choose the IMU interval.
    Event push(RawScan &&scan, double stamp, double duration);
    Event push(const ImuSample &sample);

    /// Hands over the oldest scan once every IMU sample spanning it has arrived.
    bool pop(BufferedScan &scan, std::vector<ImuSample> &imu);

    std::size_t pendingScans() const;

private:
    void resetLocked();
    mutable std::mutex mutex_;
    Preprocessor preprocessor_;
    std::deque<BufferedScan> scans_;
    std::deque<ImuSample> imu_;
    double last_imu_time_ = -std::numeric_limits<double>::infinity();
    double last_scan_time_ = -std::numeric_limits<double>::infinity();
    std::uint64_t epoch_ = 0;

    /// Running mean scan duration, used when a scan's own timing is implausible.
    double mean_duration_ = 0.1;
    int duration_samples_ = 0;
};

}  // namespace genz_lio
