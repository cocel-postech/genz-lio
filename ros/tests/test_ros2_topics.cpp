#include "OdometryServer.hpp"

#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        if (argc < 2) throw std::runtime_error("configuration directory required");
        const std::string config_dir = argv[1];
        for (int mode = 0; mode < 3; ++mode) {
            rclcpp::NodeOptions options;
            if (mode == 1) {
                options.parameter_overrides({rclcpp::Parameter("lidar_topic", ""),
                                             rclcpp::Parameter("imu_topic", "")});
            } else if (mode == 2) {
                options.parameter_overrides({rclcpp::Parameter("lidar_topic", "/test/points"),
                                             rclcpp::Parameter("imu_topic", "/test/imu")});
            }
            auto node = std::make_shared<rclcpp::Node>("topic_test_" + std::to_string(mode), options);
            genz_lio::ros_wrapper::OdometryServer server(
                node.get(), config_dir + "/default/velodyne.yaml", "");
            const auto expected_lidar = mode == 2 ? "/test/points" : "/velodyne_points";
            const auto expected_imu = mode == 2 ? "/test/imu" : "/imu/data";
            if (server.lidarTopic() != expected_lidar || server.imuTopic() != expected_imu)
                throw std::runtime_error("launch topic override did not preserve YAML fallback");
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
