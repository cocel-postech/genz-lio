// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// ROS 2 entry point.
#include "OdometryServer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <exception>
#include <thread>

namespace genz_lio {
namespace ros_wrapper {

/// Owns the server and the subscriptions. Composition rather than inheritance,
/// so that OdometryServer itself stays free of any rclcpp base class and can be
/// shared with the ROS 1 front end unchanged.
class GenZLioNode : public rclcpp::Node {
public:

    GenZLioNode() : rclcpp::Node("genz_lio") {
        const auto config_path = declare_parameter<std::string>("config_path", "");
        const auto sensor_config_path = declare_parameter<std::string>("sensor_config_path", "");

        server_ = std::make_unique<OdometryServer>(this, config_path, sensor_config_path);

        // rclcpp::SensorDataQoS would keep only the last 5 messages and drop the
        // rest whenever the pipeline falls behind. For odometry a dropped scan
        // is a hole in the LiDAR-IMU pairing, so the default is reliable with a
        // deep queue; a driver that only publishes best-effort needs
        // common/qos_reliability switched over to connect.
        rclcpp::QoS qos(rclcpp::KeepLast(server_->qosDepth()));
        if (server_->qosReliable()) {
            qos.reliable();
        } else {
            qos.best_effort();
        }

        if (server_->expectsLivoxCustomMsg()) {
#ifdef GENZ_LIO_WITH_LIVOX
            livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                server_->lidarTopic(), qos,
                [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
                    server_->onLivox(*msg);
                });
#else
            throw std::runtime_error(
                "lidar_type is 'livox', which needs the livox_ros_driver2 CustomMsg, but GenZ-LIO "
                "was built without it. Install and source livox_ros_driver2 and rebuild, or "
                "publish the sensor as PointCloud2 and set lidar_type to 'livox_pcl'.");
#endif
        } else {
            cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                server_->lidarTopic(), qos,
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    server_->onCloud(*msg);
                });
        }
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            server_->imuTopic(), qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) { server_->onImu(*msg); });

        // The processing group belongs to a dedicated executor. Do not add it
        // automatically to the input executor when the node is added there.
        drain_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { server_->spinOnce(); },
                                   drain_group_);
    }

    rclcpp::CallbackGroup::SharedPtr drainCallbackGroup() const { return drain_group_; }

private:
    std::unique_ptr<OdometryServer> server_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
#ifdef GENZ_LIO_WITH_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
#endif
    rclcpp::CallbackGroup::SharedPtr drain_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros_wrapper
}  // namespace genz_lio

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<genz_lio::ros_wrapper::GenZLioNode>();
        // Keep reception independent of scan processing, as in the ROS 1
        // wrapper. A general worker pool adds scheduler contention and lets
        // the stateful estimator migrate between workers on successive scans.
        rclcpp::executors::SingleThreadedExecutor input_executor;
        rclcpp::executors::SingleThreadedExecutor processing_executor;
        input_executor.add_node(node);
        processing_executor.add_callback_group(node->drainCallbackGroup(), node->get_node_base_interface());

        std::exception_ptr processing_error;
        std::thread processing_thread([&] {
            try {
                processing_executor.spin();
            } catch (...) {
                processing_error = std::current_exception();
                // Shutdown also covers an error before input spin() starts.
                rclcpp::shutdown();
            }
        });
        try {
            input_executor.spin();
        } catch (...) {
            rclcpp::shutdown();
            processing_thread.join();
            throw;
        }
        processing_executor.cancel();
        processing_thread.join();
        if (processing_error) std::rethrow_exception(processing_error);
    } catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("genz_lio"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
