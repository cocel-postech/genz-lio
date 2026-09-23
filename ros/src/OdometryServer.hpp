// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// The ROS front end, shared by both versions. It buffers the two streams, pairs
// each scan with the inertial samples spanning it, hands the pair to the
// pipeline and publishes what comes back. All of the estimation lives in
// genz_lio::GenZLIO; nothing here does arithmetic on a pose.
#pragma once

#include "RosCompat.hpp"
#include "InputDiagnostics.hpp"
#include "ScanBuffer.hpp"
#include "TerminalStatus.hpp"
#include "core/GenZLIO.hpp"

#ifdef GENZ_LIO_WITH_LIVOX
#if GENZ_LIO_ROS_VERSION == 2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#else
#include <livox_ros_driver/CustomMsg.h>
#endif
#endif

#include <memory>
#include <string>
#include <vector>

namespace genz_lio {
namespace ros_wrapper {

/// Publishing and recording options, which belong to the node rather than the
/// estimator.
struct OutputConfig {
    bool terminal_status_en = false;
    bool path_en = true;
    bool scan_publish_en = true;
    bool dense_publish_en = false;
    bool scan_bodyframe_pub_en = false;
    bool scan_semantic_publish_en = false;
    bool voxel_map_en = false;
    int voxel_map_max_layer = 1;

    bool pcd_save_en = false;
    int pcd_save_interval = -1;
    std::string pcd_directory;

    std::string odom_frame = "camera_init";
    std::string body_frame = "body";
    std::string world_frame = "world";
};

class OdometryServer {
public:
    /// `config_path` is the base configuration; `sensor_config_path`, when given,
    /// is layered over it.
    OdometryServer(Node *node, const std::string &config_path,
                   const std::string &sensor_config_path);
    ~OdometryServer();

    /// Drains whatever complete scan/IMU pairs the buffers now hold.
    void spinOnce();

    const std::string &lidarTopic() const { return lidar_topic_; }
    const std::string &imuTopic() const { return imu_topic_; }

    /// ROS 2 input reliability. A reliable subscription requires a reliable
    /// publisher; use best-effort for drivers that publish best-effort. Both
    /// modes have finite history, so reliability alone does not ensure complete
    /// LiDAR/IMU reception when callbacks or transport fall behind.
    bool qosReliable() const { return qos_reliable_; }
    int qosDepth() const { return qos_depth_; }
    bool expectsLivoxCustomMsg() const { return config_.preprocess.lidar_type == LidarType::Livox; }

    // Callbacks, wired up by the version-specific main.
    void onCloud(const PointCloud2 &msg);
    void onImu(const Imu &msg);
#ifdef GENZ_LIO_WITH_LIVOX
#if GENZ_LIO_ROS_VERSION == 2
    void onLivox(const livox_ros_driver2::msg::CustomMsg &msg);
#else
    void onLivox(const livox_ros_driver::CustomMsg &msg);
#endif
#endif

private:
    void readNodeOptions(const std::string &config_path, const std::string &sensor_config_path);
    void advertise();
    void publish(const GenZLIO::Result &result, const Time &stamp);
    void publishCloud(const GenZLIO::Result &result, const Time &stamp);
    void publishSemanticCloud(const Time &stamp);
    void publishPoints(const Publisher<PointCloud2> &publisher, const std::vector<Vec3> &points,
                       const Time &stamp, const std::string &frame) const;
    void accumulateForPcd(const GenZLIO::Result &result);
    void flushPcd();

    Node *node_ = nullptr;
    Config config_;
    OutputConfig output_;
    TerminalStatus terminal_status_;
    std::unique_ptr<GenZLIO> pipeline_;
    ScanBuffer buffer_;
    std::unique_ptr<InputDiagnostics> diagnostics_;
    std::uint64_t pipeline_epoch_ = 0;

    std::string lidar_topic_ = "/points";
    std::string imu_topic_ = "/imu";
    double time_offset_ = 0.0;
    /// ROS 2 only: how the subscriptions ask for their data. See qosReliable().
    bool qos_reliable_ = true;
    int qos_depth_ = 200;

    Publisher<Odometry> odom_pub_;
    Publisher<Path> path_pub_;
    Publisher<PointCloud2> cloud_world_pub_;
    Publisher<PointCloud2> cloud_body_pub_;
    Publisher<PointCloud2> planar_pub_;
    Publisher<PointCloud2> non_planar_pub_;
    Publisher<MarkerArray> voxel_map_pub_;
    std::unique_ptr<TfBroadcaster> tf_broadcaster_;
    std::unique_ptr<StaticTfBroadcaster> static_tf_broadcaster_;
    bool gravity_transform_sent_ = false;

    Path path_;
    std::vector<Vec3> pcd_points_;
    int pcd_index_ = 0;
    int pcd_frames_ = 0;
};

}  // namespace ros_wrapper
}  // namespace genz_lio
