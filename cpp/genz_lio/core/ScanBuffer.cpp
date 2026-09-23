// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "ScanBuffer.hpp"

#include <cmath>

namespace genz_lio {

void ScanBuffer::resetLocked() {
    scans_.clear();
    imu_.clear();
    last_scan_time_ = last_imu_time_ = -std::numeric_limits<double>::infinity();
    mean_duration_ = 0.1;
    duration_samples_ = 0;
    ++epoch_;
}

void ScanBuffer::setPreprocessConfig(const PreprocessConfig &config) {
    std::lock_guard<std::mutex> lock(mutex_);
    preprocessor_ = Preprocessor(config);
}

ScanBuffer::Event ScanBuffer::push(RawScan &&scan, const double stamp, const double /*duration*/) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(stamp) || !preprocessor_.prepareTiming(scan)) return Event::InvalidScan;
    // Some drivers stamp the message at the end of the sweep and report times
    // running back to its start. Anchoring on the reported times handles that
    // and the forward convention alike, and keeps the window aligned with the
    // IMU samples that actually span it.
    const double begin_time = stamp + scan.time_min;

    Event event = Event::None;
    if (begin_time < last_scan_time_) {
        resetLocked();
        event = Event::ScanStampsWentBackwards;
    }

    double span = scan.time_max - scan.time_min;
    if (scan.points.size() <= 1 || span < 0.5 * mean_duration_) {
        span = mean_duration_;
    } else {
        ++duration_samples_;
        mean_duration_ += (span - mean_duration_) / duration_samples_;
    }

    last_scan_time_ = begin_time;

    BufferedScan buffered;
    buffered.scan = std::move(scan);
    buffered.begin_time = begin_time;
    buffered.end_time = begin_time + span;
    buffered.epoch = epoch_;
    scans_.push_back(std::move(buffered));
    return event;
}

ScanBuffer::Event ScanBuffer::push(const ImuSample &sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(sample.timestamp) || !sample.linear_acceleration.allFinite() ||
        !sample.angular_velocity.allFinite()) return Event::InvalidImu;
    Event event = Event::None;
    if (sample.timestamp < last_imu_time_) {
        resetLocked();
        event = Event::ImuStampsWentBackwards;
    }
    last_imu_time_ = sample.timestamp;
    imu_.push_back(sample);
    return event;
}

bool ScanBuffer::pop(BufferedScan &scan, std::vector<ImuSample> &imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (scans_.empty() || imu_.empty()) return false;
    // Wait until the IMU has caught up with the end of the oldest scan.
    if (last_imu_time_ < scans_.front().end_time) return false;

    scan = std::move(scans_.front());
    scans_.pop_front();

    imu.clear();
    while (!imu_.empty() && imu_.front().timestamp <= scan.end_time) {
        imu.push_back(imu_.front());
        imu_.pop_front();
    }
    return true;
}

std::size_t ScanBuffer::pendingScans() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return scans_.size();
}

}  // namespace genz_lio
