// This file is part of GenZ-LIO, released under the GNU GPL v2.
#pragma once

#include "core/GenZLIO.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <unistd.h>

namespace genz_lio::ros_wrapper {

/// Read-only display state. Never touches the estimator or its configuration.
class TerminalStatus {
public:
    void reset() { processing_ms_ = 0.; frames_ = 0; }

    std::string format(const GenZLIO::Result &result, double scale_threshold, bool interactive) {
        if (std::isfinite(result.processing_time_ms) && result.processing_time_ms > 0.) {
            processing_ms_ += result.processing_time_ms;
            ++frames_;
        }
        const double fps = processing_ms_ > 0. ? 1000. * frames_ / processing_ms_ : 0.;
        const char *red = interactive ? "\033[1;31m" : "";
        const char *blue = interactive ? "\033[1;34m" : "";
        const char *green = interactive ? "\033[1;32m" : "";
        const char *normal = interactive ? "\033[0m" : "";

        std::ostringstream scale;
        scale.imbue(std::locale::classic());
        scale << "Scale indicator: " << std::fixed << std::setprecision(2) << result.scale_indicator;
        const std::string non_planar = "# of non-planar points: " + std::to_string(result.point_matches);
        const std::string planar = "# of planar points: " + std::to_string(result.plane_matches);
        const std::size_t count_width = non_planar.size() + 2 + planar.size();
        const std::size_t minimum_width = std::string("Confined <----- ").size()
                                       + scale.str().size() + std::string(" -----> Open").size();
        const std::size_t width = std::max(count_width, minimum_width);
        const std::size_t extra = width - minimum_width;
        const std::string left = "Confined <" + std::string(5 + extra / 2, '-') + " ";
        const std::string right = " " + std::string(5 + extra - extra / 2, '-') + "> Open";
        const int dashes = static_cast<int>(width) - 4;
        double fraction = 0.;
        if (std::isfinite(result.scale_indicator) && std::isfinite(scale_threshold) && scale_threshold > 0.)
            fraction = std::clamp(result.scale_indicator / scale_threshold, 0., 1.);
        const int marker = static_cast<int>(std::lround(fraction * dashes));

        std::ostringstream out;
        out.imbue(std::locale::classic());
        if (interactive) out << "\033[2J\033[H";
        out << "====================== GenZ-LIO ======================\n"
            << red << non_planar << normal << ", " << std::string(width - count_width, ' ')
            << blue << planar << normal << '\n'
            << left << green << scale.str() << normal << right << '\n'
            << '[' << std::string(marker, '-') << green << "[]" << normal
            << std::string(dashes - marker, '-') << "]\n"
            << "# of target points: " << std::fixed << std::setprecision(0) << result.setpoint << '\n'
            << "Adaptive voxel size: " << std::setprecision(2) << result.leaf_size << " m\n"
            << "# of raw points: " << result.deskewed.size() << '\n'
            << "# of voxelized points: " << result.voxelized_points << '\n'
            << "Processing time: " << result.processing_time_ms << " ms\n"
            << "FPS: " << fps << '\n';
        return out.str();
    }

    void print(const GenZLIO::Result &result, double scale_threshold) {
        const auto text = format(result, scale_threshold, ::isatty(::fileno(stdout)) != 0);
        // One buffered write per valid frame. Files/pipes get plain append-only
        // blocks, not terminal erase/color sequences.
        std::fwrite(text.data(), 1, text.size(), stdout);
        std::fflush(stdout);
    }

private:
    double processing_ms_ = 0.;
    std::size_t frames_ = 0;
};

}  // namespace genz_lio::ros_wrapper
