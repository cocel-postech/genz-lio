// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Loading a Config from YAML. Kept out of the core so that the core itself needs
// nothing beyond Eigen, and shared by the ROS 1 and ROS 2 wrappers so that both
// read the very same configuration files.
#pragma once

#include "core/Config.hpp"

#include <string>
#include <vector>

namespace genz_lio {

/// Parses `path` on top of the shipped core defaults. Any key the file omits
/// keeps its default; use explicit experiment configs for paper reproduction.
///
/// Throws std::runtime_error if the file cannot be read or a value has the wrong
/// shape. Unknown keys are collected into `warnings` rather than rejected, so
/// that a stale configuration still runs while saying what it got wrong.
Config loadConfig(const std::string &path, std::vector<std::string> *warnings = nullptr);

/// Applies a custom/legacy partial override on top of existing values.
/// Shipped config/default/<sensor>.yaml files are complete; use loadConfig.
void loadConfigInto(const std::string &path, Config &config,
                    std::vector<std::string> *warnings = nullptr);

/// Renders a config back to YAML, in the same layout the shipped files use.
std::string dumpConfig(const Config &config);

}  // namespace genz_lio
