// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "config/ConfigIo.hpp"
#include "Check.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

namespace {

std::string writeTemp(const std::string &name, const std::string &contents) {
    const std::string path = std::string("/tmp/genz_config_") + name + ".yaml";
    std::ofstream(path) << contents;
    return path;
}

void shippedDefaultsMatchCoreDefaults() {
    const char *root = std::getenv("GENZ_LIO_CONFIG_DIR");
    if (!root) {
        std::printf("  (GENZ_LIO_CONFIG_DIR unset, skipping shipped-config check)\n");
        return;
    }
    std::vector<std::string> warnings;
    const auto config = genz_lio::loadConfig(std::string(root) + "/default/velodyne.yaml", &warnings);

    for (const auto &warning : warnings) std::printf("  warning: %s\n", warning.c_str());
    GENZ_CHECK(warnings.empty());  // the shipped file must not carry stale keys

    const auto &av = config.adaptive_voxelization;
    GENZ_CHECK(av.enable);
    GENZ_CHECK(av.window_size == 5);
    GENZ_CHECK(av.scale_threshold == 30.0);
    GENZ_CHECK(av.setpoint_exponent == 2);
    GENZ_CHECK(av.min_points == 1000 && av.max_points == 4000);
    GENZ_CHECK(av.error_sensitivity == 0.1 && av.error_rate_sensitivity == 0.2);
    GENZ_CHECK(av.p_gain_min == 5e-6 && av.p_gain_max == 5e-5);
    GENZ_CHECK(av.d_gain_min == 5e-8 && av.d_gain_max == 5e-7);

    GENZ_CHECK(config.hybrid_metric.enable);
    GENZ_CHECK(config.hybrid_metric.lambda_po == 0.05);
    const genz_lio::Config defaults;
    GENZ_CHECK(config.mapping.voxel_size == 1.0);
    GENZ_CHECK(config.mapping.max_layer == 4);
    GENZ_CHECK(config.mapping.planar_threshold == 0.001);
    GENZ_CHECK(config.hybrid_metric.max_points_per_voxel == 128);
    GENZ_CHECK(config.mapping.voxel_size == defaults.mapping.voxel_size);
    GENZ_CHECK(config.mapping.max_layer == defaults.mapping.max_layer);
    GENZ_CHECK(config.mapping.planar_threshold == defaults.mapping.planar_threshold);
    GENZ_CHECK(config.hybrid_metric.max_points_per_voxel == defaults.hybrid_metric.max_points_per_voxel);
}

void completeSensorDefaultsLoadIndependently() {
    const char *root = std::getenv("GENZ_LIO_CONFIG_DIR");
    if (!root) return;

    for (const auto *name : {"avia", "hesai", "mid360", "ouster", "robosense", "velodyne"}) {
        std::vector<std::string> warnings;
        const auto config = genz_lio::loadConfig(
            std::string(root) + "/default/" + name + ".yaml", &warnings);
        GENZ_CHECK(warnings.empty());
        GENZ_CHECK(config.mapping.voxel_size == 1.0);
        GENZ_CHECK(config.mapping.max_layer == 4);
        GENZ_CHECK(config.mapping.planar_threshold == 0.001);
        GENZ_CHECK(config.hybrid_metric.max_points_per_voxel == 128);
        GENZ_CHECK(config.hybrid_metric.lambda_po == 0.05);
        GENZ_CHECK(config.adaptive_voxelization.scale_threshold == 30.0);
    }
    const auto ouster = genz_lio::loadConfig(std::string(root) + "/default/ouster.yaml");
    GENZ_CHECK(ouster.preprocess.lidar_type == genz_lio::LidarType::Ouster);
    GENZ_CHECK(ouster.preprocess.scan_line == 64);
}

void omittedKeysKeepTheirDefaults() {
    const auto path = writeTemp("partial", "hybrid_metric:\n    lambda_po: 0.2\n");
    const auto config = genz_lio::loadConfig(path);
    GENZ_CHECK(config.hybrid_metric.lambda_po == 0.2);
    GENZ_CHECK(config.adaptive_voxelization.max_points == 4000);  // untouched
    GENZ_CHECK(config.mapping.voxel_size == 1.0);
}

void unknownKeysWarnRatherThanFail() {
    const auto path = writeTemp(
        "stale", "mapping:\n    fov_degree: 360\n    det_range: 100.0\nlegacy_section:\n    a: 1\n");
    std::vector<std::string> warnings;
    const auto config = genz_lio::loadConfig(path, &warnings);
    GENZ_CHECK(warnings.size() == 3);
    GENZ_CHECK(config.mapping.voxel_size == 1.0);  // still usable
}

void badValuesAreRejected() {
    bool threw = false;
    try {
        genz_lio::loadConfig(writeTemp("badlidar", "preprocess:\n    lidar_type: sick\n"));
    } catch (const std::runtime_error &) {
        threw = true;
    }
    GENZ_CHECK(threw);

    threw = false;
    try {
        genz_lio::loadConfig(writeTemp("badextrinsic", "mapping:\n    extrinsic_t: [1.0, 2.0]\n"));
    } catch (const std::runtime_error &) {
        threw = true;
    }
    GENZ_CHECK(threw);

    threw = false;
    try {
        genz_lio::loadConfig("/tmp/genz_config_does_not_exist.yaml");
    } catch (const std::runtime_error &) {
        threw = true;
    }
    GENZ_CHECK(threw);
}

void extrinsicsAreReadRowMajor() {
    const auto path = writeTemp("extrinsic",
                                "mapping:\n"
                                "    extrinsic_t: [0.1, 0.2, 0.3]\n"
                                "    extrinsic_r: [1.0, 0.0, 0.0,\n"
                                "                  0.0, -1.0, 0.0,\n"
                                "                  0.0, 0.0, -1.0]\n");
    const auto config = genz_lio::loadConfig(path);
    GENZ_CHECK((config.mapping.extrinsic_t - genz_lio::Vec3(0.1, 0.2, 0.3)).norm() < 1e-12);
    GENZ_CHECK(config.mapping.extrinsic_r(1, 1) == -1.0);
    GENZ_CHECK(config.mapping.extrinsic_r(2, 2) == -1.0);
    GENZ_CHECK(config.mapping.extrinsic_r(0, 0) == 1.0);
}

}  // namespace

int main() {
    shippedDefaultsMatchCoreDefaults();
    completeSensorDefaultsLoadIndependently();
    omittedKeysKeepTheirDefaults();
    unknownKeysWarnRatherThanFail();
    badValuesAreRejected();
    extrinsicsAreReadRowMajor();
    std::printf("config io ok\n");
    return 0;
}
