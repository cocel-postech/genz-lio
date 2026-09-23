#include "TerminalStatus.hpp"
#include <iostream>
#include <limits>
#include <regex>
#include <stdexcept>
#include <vector>

using genz_lio::GenZLIO;
using genz_lio::ros_wrapper::TerminalStatus;
void require(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}
std::vector<std::string> lines(const std::string &text) {
    std::istringstream input(text);
    std::vector<std::string> result;
    for (std::string line; std::getline(input, line);) result.push_back(line);
    return result;
}
int main() {
    try {
        GenZLIO::Result result;
        result.point_matches = 12; result.plane_matches = 234;
        result.leaf_size = .123456; result.setpoint = 2345.67;
        result.voxelized_points = 246; result.deskewed.resize(1000);
        result.processing_time_ms = 10.; result.scale_indicator = 12.34;
        TerminalStatus status;
        auto plain = status.format(result, 30., false);
        require(plain.find('\033') == std::string::npos, "redirected output contains ANSI");
        require(plain.find("Odometry information") == std::string::npos, "redundant terminal heading");
        require(plain.find("# of target points: 2346\n") != std::string::npos, "target integer precision");
        require(plain.find("Adaptive voxel size: 0.12 m\n") != std::string::npos, "voxel precision");
        require(plain.find("# of raw points: 1000\n") != std::string::npos, "raw count is not deskewed input");
        require(plain.find("Processing time: 10.00 ms\nFPS: 100.00\n") != std::string::npos, "time/FPS layout");
        result.processing_time_ms = 30.;
        auto tty = status.format(result, 30., true);
        require(tty.rfind("\033[2J\033[H", 0) == 0, "terminal is not cleared before refresh");
        require(tty.find("\033[1;32m[]\033[0m") != std::string::npos, "green [] marker");
        require(tty.find("FPS: 50.00\n") != std::string::npos, "FPS must use mean processing time");
        auto stripped = std::regex_replace(tty, std::regex("\033\\[[0-9;]*[A-Za-z]"), "");
        const auto split = lines(stripped);
        require(split[1].size() == split[2].size(), "Open does not align with final planar count digit");
        require(split[2].size() == split[3].size(), "bar does not align with Open");
        require(split[3].find(' ') == std::string::npos, "bar contains inserted spaces");
        for (int count : {0, 9, 1234, 123456}) {
            result.point_matches = count; result.plane_matches = count;
            const auto display = lines(status.format(result, 30., false));
            require(display[1].size() == display[2].size() && display[2].size() == display[3].size(),
                    "alignment changes with point-count digit width");
        }
        for (double scale : {-1., 0., 30., 90., std::numeric_limits<double>::quiet_NaN()}) {
            result.scale_indicator = scale;
            const auto bar = lines(status.format(result, 30., false))[3];
            require(bar.front() == '[' && bar.back() == ']' && bar.find("[]") != std::string::npos, "missing marker at scale boundary");
            if (scale <= 0.) require(bar.rfind("[[]", 0) == 0, "confined marker position");
            if (scale >= 30.) require(bar.substr(bar.size()-3) == "[]]", "open marker position");
        }
        status.reset(); result.processing_time_ms = 0.;
        require(status.format(result, 0., false).find("FPS: 0.00\n") != std::string::npos, "FPS reset/zero threshold");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n'; return 1;
    }
    return 0;
}
