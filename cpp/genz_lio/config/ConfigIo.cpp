// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "ConfigIo.hpp"

#include <yaml-cpp/yaml.h>

#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace genz_lio {
namespace {

const std::map<std::string, LidarType> kLidarTypes = {
    {"livox", LidarType::Livox},         {"velodyne", LidarType::Velodyne},
    {"ouster", LidarType::Ouster},       {"livox_pcl", LidarType::LivoxPcl},
    {"hesai", LidarType::Hesai},         {"robosense", LidarType::Robosense},
};

/// Sections the wrappers own rather than the core; seeing them is not an error.
const std::set<std::string> kWrapperSections = {"common", "publish", "pcd_save"};

std::string join(const std::map<std::string, LidarType> &values) {
    std::ostringstream out;
    for (auto it = values.begin(); it != values.end(); ++it) {
        if (it != values.begin()) out << ", ";
        out << it->first;
    }
    return out.str();
}

/// Reads one scalar, leaving the destination alone when the key is absent.
template <typename T>
void read(const YAML::Node &node, const std::string &key, T &destination,
          std::set<std::string> &seen) {
    seen.insert(key);
    if (!node[key]) return;
    try {
        destination = node[key].as<T>();
    } catch (const YAML::Exception &e) {
        throw std::runtime_error("config key '" + key + "' has the wrong type: " + e.what());
    }
}

void readVector(const YAML::Node &node, const std::string &key, std::vector<double> &destination,
                const std::size_t expected, std::set<std::string> &seen) {
    seen.insert(key);
    if (!node[key]) return;
    const auto values = node[key].as<std::vector<double>>();
    if (values.size() != expected) {
        throw std::runtime_error("config key '" + key + "' expects " + std::to_string(expected) +
                                 " values, got " + std::to_string(values.size()));
    }
    destination = values;
}

void collectUnknown(const YAML::Node &section, const std::set<std::string> &known,
                    const std::string &prefix, std::vector<std::string> *warnings) {
    if (!warnings || !section.IsMap()) return;
    for (const auto &entry : section) {
        const auto key = entry.first.as<std::string>();
        if (!known.count(key)) warnings->push_back("unknown key '" + prefix + key + "'");
    }
}

void loadPreprocess(const YAML::Node &node, PreprocessConfig &config,
                    std::vector<std::string> *warnings) {
    std::set<std::string> seen;
    seen.insert("lidar_type");
    if (node["lidar_type"]) {
        const auto name = node["lidar_type"].as<std::string>();
        const auto found = kLidarTypes.find(name);
        if (found == kLidarTypes.end()) {
            throw std::runtime_error("unknown lidar_type '" + name + "'; expected one of " +
                                     join(kLidarTypes));
        }
        config.lidar_type = found->second;
    }
    read(node, "scan_line", config.scan_line, seen);
    read(node, "scan_rate", config.scan_rate, seen);
    read(node, "blind_min", config.blind_min, seen);
    read(node, "blind_max", config.blind_max, seen);
    read(node, "point_filter_num", config.point_filter_num, seen);
    collectUnknown(node, seen, "preprocess/", warnings);
}

void loadMapping(const YAML::Node &node, MappingConfig &config, std::vector<std::string> *warnings) {
    std::set<std::string> seen;
    read(node, "max_iteration", config.max_iteration, seen);
    read(node, "down_sample_size", config.down_sample_size, seen);
    read(node, "voxel_size", config.voxel_size, seen);
    read(node, "max_layer", config.max_layer, seen);
    read(node, "planar_threshold", config.planar_threshold, seen);
    read(node, "sigma_num", config.sigma_num, seen);
    read(node, "max_points_size", config.max_points_size, seen);
    read(node, "max_mature_points_size", config.max_mature_points_size, seen);
    read(node, "map_range", config.map_range, seen);
    read(node, "extrinsic_est_en", config.extrinsic_est_en, seen);

    seen.insert("layer_point_size");
    if (node["layer_point_size"]) {
        config.layer_point_size = node["layer_point_size"].as<std::vector<int>>();
    }

    std::vector<double> translation{0, 0, 0};
    std::vector<double> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
    readVector(node, "extrinsic_t", translation, 3, seen);
    readVector(node, "extrinsic_r", rotation, 9, seen);
    config.extrinsic_t = Vec3(translation[0], translation[1], translation[2]);
    config.extrinsic_r << rotation[0], rotation[1], rotation[2], rotation[3], rotation[4],
        rotation[5], rotation[6], rotation[7], rotation[8];

    collectUnknown(node, seen, "mapping/", warnings);
}

void loadAdaptiveVoxelization(const YAML::Node &node, AdaptiveVoxelizationConfig &config,
                              std::vector<std::string> *warnings) {
    std::set<std::string> seen;
    read(node, "enable", config.enable, seen);
    read(node, "window_size", config.window_size, seen);
    read(node, "scale_threshold", config.scale_threshold, seen);
    read(node, "setpoint_exponent", config.setpoint_exponent, seen);
    read(node, "min_points", config.min_points, seen);
    read(node, "max_points", config.max_points, seen);
    read(node, "error_sensitivity", config.error_sensitivity, seen);
    read(node, "error_rate_sensitivity", config.error_rate_sensitivity, seen);
    read(node, "p_gain_min", config.p_gain_min, seen);
    read(node, "p_gain_max", config.p_gain_max, seen);
    read(node, "d_gain_min", config.d_gain_min, seen);
    read(node, "d_gain_max", config.d_gain_max, seen);
    collectUnknown(node, seen, "adaptive_voxelization/", warnings);
}

void loadHybridMetric(const YAML::Node &node, HybridMetricConfig &config,
                      std::vector<std::string> *warnings) {
    std::set<std::string> seen;
    read(node, "enable", config.enable, seen);
    read(node, "lambda_po", config.lambda_po, seen);
    read(node, "sigma_num", config.sigma_num, seen);
    read(node, "max_points_per_voxel", config.max_points_per_voxel, seen);
    read(node, "reduction_ratio", config.reduction_ratio, seen);

    seen.insert("adaptive_threshold");
    if (const auto threshold = node["adaptive_threshold"]) {
        std::set<std::string> threshold_seen;
        auto &target = config.adaptive_threshold;
        read(threshold, "initial_threshold", target.initial_threshold, threshold_seen);
        read(threshold, "max_range_motion", target.max_range_motion, threshold_seen);
        collectUnknown(threshold, threshold_seen, "hybrid_metric/adaptive_threshold/", warnings);
    }
    collectUnknown(node, seen, "hybrid_metric/", warnings);
}

void loadNoiseModel(const YAML::Node &node, NoiseModelConfig &config,
                    std::vector<std::string> *warnings) {
    std::set<std::string> seen;
    read(node, "ranging_cov", config.ranging_cov, seen);
    read(node, "angle_cov", config.angle_cov, seen);
    read(node, "acc_cov", config.acc_cov, seen);
    read(node, "gyr_cov", config.gyr_cov, seen);
    read(node, "b_acc_cov", config.b_acc_cov, seen);
    read(node, "b_gyr_cov", config.b_gyr_cov, seen);
    collectUnknown(node, seen, "noise_model/", warnings);
}

}  // namespace

void loadConfigInto(const std::string &path, Config &config,
                    std::vector<std::string> *warnings) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception &e) {
        throw std::runtime_error("cannot read config '" + path + "': " + e.what());
    }
    if (!root.IsMap()) throw std::runtime_error("config '" + path + "' is not a mapping");

    if (const auto runtime = root["runtime"]) {
        std::set<std::string> seen;
        read(runtime, "max_threads", config.max_threads, seen);
        collectUnknown(runtime, seen, "runtime/", warnings);
    }
    if (root["preprocess"]) loadPreprocess(root["preprocess"], config.preprocess, warnings);
    if (root["mapping"]) loadMapping(root["mapping"], config.mapping, warnings);
    if (root["adaptive_voxelization"]) {
        loadAdaptiveVoxelization(root["adaptive_voxelization"], config.adaptive_voxelization,
                                 warnings);
    }
    if (root["hybrid_metric"]) loadHybridMetric(root["hybrid_metric"], config.hybrid_metric, warnings);
    if (root["noise_model"]) loadNoiseModel(root["noise_model"], config.noise_model, warnings);

    if (warnings) {
        const std::set<std::string> known = {"runtime", "preprocess", "mapping",
                                             "adaptive_voxelization", "hybrid_metric",
                                             "noise_model"};
        for (const auto &entry : root) {
            const auto key = entry.first.as<std::string>();
            if (!known.count(key) && !kWrapperSections.count(key)) {
                warnings->push_back("unknown section '" + key + "'");
            }
        }
    }
}

Config loadConfig(const std::string &path, std::vector<std::string> *warnings) {
    Config config;  // shipped algorithm defaults
    loadConfigInto(path, config, warnings);
    return config;
}

std::string dumpConfig(const Config &config) {
    auto lidar_name = [&] {
        for (const auto &entry : kLidarTypes) {
            if (entry.second == config.preprocess.lidar_type) return entry.first;
        }
        return std::string("velodyne");
    };

    std::ostringstream out;
    out << "preprocess:\n"
        << "    lidar_type: " << lidar_name() << "\n"
        << "    scan_line: " << config.preprocess.scan_line << "\n"
        << "    scan_rate: " << config.preprocess.scan_rate << "\n"
        << "    blind_min: " << config.preprocess.blind_min << "\n"
        << "    blind_max: " << config.preprocess.blind_max << "\n"
        << "    point_filter_num: " << config.preprocess.point_filter_num << "\n\n";

    out << "mapping:\n"
        << "    max_iteration: " << config.mapping.max_iteration << "\n"
        << "    down_sample_size: " << config.mapping.down_sample_size << "\n"
        << "    voxel_size: " << config.mapping.voxel_size << "\n"
        << "    max_layer: " << config.mapping.max_layer << "\n"
        << "    planar_threshold: " << config.mapping.planar_threshold << "\n"
        << "    sigma_num: " << config.mapping.sigma_num << "\n"
        << "    max_points_size: " << config.mapping.max_points_size << "\n"
        << "    max_mature_points_size: " << config.mapping.max_mature_points_size << "\n"
        << "    map_range: " << config.mapping.map_range << "\n"
        << "    extrinsic_est_en: " << (config.mapping.extrinsic_est_en ? "true" : "false")
        << "\n\n";

    out << "adaptive_voxelization:\n"
        << "    enable: " << (config.adaptive_voxelization.enable ? "true" : "false") << "\n"
        << "    window_size: " << config.adaptive_voxelization.window_size << "\n"
        << "    scale_threshold: " << config.adaptive_voxelization.scale_threshold << "\n"
        << "    setpoint_exponent: " << config.adaptive_voxelization.setpoint_exponent << "\n"
        << "    min_points: " << config.adaptive_voxelization.min_points << "\n"
        << "    max_points: " << config.adaptive_voxelization.max_points << "\n\n";

    out << "hybrid_metric:\n"
        << "    enable: " << (config.hybrid_metric.enable ? "true" : "false") << "\n"
        << "    lambda_po: " << config.hybrid_metric.lambda_po << "\n";
    return out.str();
}

}  // namespace genz_lio
