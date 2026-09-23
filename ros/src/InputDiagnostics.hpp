// This file is part of GenZ-LIO, released under the GNU GPL v2.
#pragma once

#include <chrono>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>

namespace genz_lio {
namespace ros_wrapper {

/// Optional receiver-side trace. One file per node; callbacks may use it concurrently.
/// Buffered writes avoid flushing once per IMU message. Shutdown closes the file.
class InputDiagnostics {
public:
    explicit InputDiagnostics(const char *path) : file_(std::fopen(path, "w")) {
        if (!file_) throw std::runtime_error(std::string("cannot open GENZ_LIO_DIAGNOSTICS: ") + path);
        std::fprintf(file_, "event,stamp,end,count,first,last,status,aux,wall_seconds\n");
    }
    ~InputDiagnostics() { std::fclose(file_); }
    InputDiagnostics(const InputDiagnostics &) = delete;
    InputDiagnostics &operator=(const InputDiagnostics &) = delete;

    void record(const char *event, double stamp = 0, double end = 0,
                std::size_t count = 0, double first = 0, double last = 0,
                int status = 0, std::size_t aux = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double elapsed = std::chrono::duration<double>(Clock::now() - started_).count();
        std::fprintf(file_, "%s,%.17g,%.17g,%zu,%.17g,%.17g,%d,%zu,%.9f\n",
                     event, stamp, end, count, first, last, status, aux, elapsed);
    }

private:
    using Clock = std::chrono::steady_clock;
    FILE *file_;
    std::mutex mutex_;
    Clock::time_point started_ = Clock::now();
};

}  // namespace ros_wrapper
}  // namespace genz_lio
