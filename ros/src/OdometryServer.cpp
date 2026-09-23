// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "OdometryServer.hpp"

#include "CloudConversion.hpp"
#include "VoxelMapVisualizer.hpp"
#include "config/ConfigIo.hpp"
#include "core/StateTransforms.hpp"
#include "core/Downsample.hpp"

#include <cstdio>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <sys/stat.h>

namespace genz_lio {
namespace ros_wrapper {
namespace {

std::vector<Vec3> toWorld(const PointCloud &cloud, const state_ikfom &state) {
    std::vector<Vec3> points;
    points.reserve(cloud.size());
    for (const auto &point : cloud) points.push_back(transformPointToWorld(state, point.position));
    return points;
}

}  // namespace

OdometryServer::OdometryServer(Node *node, const std::string &config_path,
                               const std::string &sensor_config_path)
    : node_(node) {
    if (config_path.empty()) {
        throw std::runtime_error("no config_path given; point it at a GenZ-LIO YAML file");
    }

    std::vector<std::string> warnings;
    config_ = loadConfig(config_path, &warnings);
    if (!sensor_config_path.empty()) loadConfigInto(sensor_config_path, config_, &warnings);
    buffer_.setPreprocessConfig(config_.preprocess);
    for (const auto &warning : warnings) GENZ_LOG_WARN(node_, "config: %s", warning.c_str());

    readNodeOptions(config_path, sensor_config_path);
    if (const char *path = std::getenv("GENZ_LIO_DIAGNOSTICS")) {
        if (*path) diagnostics_ = std::make_unique<InputDiagnostics>(path);
    }

    // An exactly identity extrinsic means the LiDAR and the IMU are claimed to
    // sit at the same point with the same orientation, which no real platform
    // does. It is almost always a configuration that was never filled in, and it
    // degrades accuracy quietly rather than failing outright.
    if (config_.mapping.extrinsic_t.isZero(0.0) && config_.mapping.extrinsic_r.isIdentity(0.0)) {
        GENZ_LOG_WARN(node_,
                      "extrinsic_t and extrinsic_r are exactly zero and identity. If that is not "
                      "your actual LiDAR-to-IMU calibration, set them in the sensor config: a "
                      "wrong extrinsic shows up as drift that grows with rotation, not as obvious "
                      "failure.");
    }

    if (output_.pcd_save_en && !output_.pcd_directory.empty()) {
        mkdir(output_.pcd_directory.c_str(), 0755);
    }

    pipeline_ = std::make_unique<GenZLIO>(config_);
    tf_broadcaster_ = std::make_unique<TfBroadcaster>(node_);
    static_tf_broadcaster_ = std::make_unique<StaticTfBroadcaster>(node_);
    advertise();
    path_.header.frame_id = output_.odom_frame;

    GENZ_LOG_INFO(node_, "GenZ-LIO listening on %s and %s", lidar_topic_.c_str(),
                  imu_topic_.c_str());
    GENZ_LOG_INFO(node_, "  adaptive voxelization: %s, hybrid metric: %s",
                  config_.adaptive_voxelization.enable ? "on" : "off",
                  config_.hybrid_metric.enable ? "on" : "off");
}

OdometryServer::~OdometryServer() {
    if (diagnostics_) diagnostics_->record("pending_scans", 0, 0, buffer_.pendingScans());
    flushPcd();
}

void OdometryServer::readNodeOptions(const std::string &config_path,
                                     const std::string &sensor_config_path) {
    // The publishing and topic settings live in the same file as the tuning, so
    // that one file fully describes a run.
    auto apply = [&](const YAML::Node &root) {
        if (const auto common = root["common"]) {
            if (common["lidar_topic"]) lidar_topic_ = common["lidar_topic"].as<std::string>();
            if (common["imu_topic"]) imu_topic_ = common["imu_topic"].as<std::string>();
            if (common["time_offset"]) time_offset_ = common["time_offset"].as<double>();
            if (common["qos_reliability"]) {
                const auto value = common["qos_reliability"].as<std::string>();
                if (value == "reliable") {
                    qos_reliable_ = true;
                } else if (value == "best_effort") {
                    qos_reliable_ = false;
                } else {
                    GENZ_LOG_WARN(node_, "unknown qos_reliability '%s'; expected reliable or "
                                         "best_effort, keeping reliable", value.c_str());
                }
            }
            if (common["qos_depth"]) qos_depth_ = common["qos_depth"].as<int>();
        }
        if (const auto publish = root["publish"]) {
            auto flag = [&](const char *key, bool &target) {
                if (publish[key]) target = publish[key].as<bool>();
            };
            flag("terminal_status_en", output_.terminal_status_en);
            flag("path_en", output_.path_en);
            flag("scan_publish_en", output_.scan_publish_en);
            flag("dense_publish_en", output_.dense_publish_en);
            flag("scan_bodyframe_pub_en", output_.scan_bodyframe_pub_en);
            flag("scan_semantic_publish_en", output_.scan_semantic_publish_en);
            flag("voxel_map_en", output_.voxel_map_en);
            if (publish["voxel_map_max_layer"]) {
                output_.voxel_map_max_layer = publish["voxel_map_max_layer"].as<int>();
            }
        }
        if (const auto pcd = root["pcd_save"]) {
            if (pcd["enable"]) output_.pcd_save_en = pcd["enable"].as<bool>();
            if (pcd["interval"]) output_.pcd_save_interval = pcd["interval"].as<int>();
            if (pcd["directory"]) output_.pcd_directory = pcd["directory"].as<std::string>();
        }
    };
    apply(YAML::LoadFile(config_path));
    if (!sensor_config_path.empty()) apply(YAML::LoadFile(sensor_config_path));

    // Launch files use an empty string to mean "keep the YAML topic".
    const auto lidar_override = getParameter<std::string>(node_, "lidar_topic", lidar_topic_);
    const auto imu_override = getParameter<std::string>(node_, "imu_topic", imu_topic_);
    if (!lidar_override.empty()) lidar_topic_ = lidar_override;
    if (!imu_override.empty()) imu_topic_ = imu_override;
}

void OdometryServer::advertise() {
    odom_pub_ = Publisher<Odometry>(node_, "/Odometry", 100);
    path_pub_ = Publisher<Path>(node_, "/path", 100);
    cloud_world_pub_ = Publisher<PointCloud2>(node_, "/cloud_registered", 100);
    cloud_body_pub_ = Publisher<PointCloud2>(node_, "/cloud_registered_body", 100);
    planar_pub_ = Publisher<PointCloud2>(node_, "/cloud_planar", 100);
    non_planar_pub_ = Publisher<PointCloud2>(node_, "/cloud_non_planar", 100);
    voxel_map_pub_ = Publisher<MarkerArray>(node_, "/voxel_map", 10);
}

void OdometryServer::onCloud(const PointCloud2 &msg) {
    if (diagnostics_) diagnostics_->record("cloud_received", toSeconds(msg.header.stamp), 0,
                                          static_cast<std::size_t>(msg.width) * msg.height);
    auto converted = toRawScan(msg, config_.preprocess.lidar_type);
    if (!converted.warning.empty()) {
        GENZ_LOG_WARN(node_, "point cloud: %s", converted.warning.c_str());
    }
    if (converted.scan.points.empty()) {
        if (diagnostics_) diagnostics_->record("cloud_empty", toSeconds(msg.header.stamp));
        return;
    }

    const double stamp = toSeconds(msg.header.stamp) + time_offset_;
    const auto event = buffer_.push(std::move(converted.scan), stamp, converted.duration);
    if (diagnostics_) diagnostics_->record("cloud_buffered", stamp, 0, buffer_.pendingScans(),
                                          0, 0, static_cast<int>(event));
    if (event == ScanBuffer::Event::ScanStampsWentBackwards) {
        GENZ_LOG_WARN(node_, "LiDAR stamps went backwards; cleared both streams, estimator will restart");
    } else if (event == ScanBuffer::Event::InvalidScan) {
        GENZ_LOG_WARN(node_, "LiDAR scan has no usable point timing; scan skipped");
    }
}

void OdometryServer::onImu(const Imu &msg) {
    ImuSample sample;
    sample.timestamp = toSeconds(msg.header.stamp);
    sample.linear_acceleration =
        Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    sample.angular_velocity =
        Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

    const auto event = buffer_.push(sample);
    if (diagnostics_) diagnostics_->record("imu_received", sample.timestamp, 0, 1,
                                          0, 0, static_cast<int>(event));
    if (event == ScanBuffer::Event::ImuStampsWentBackwards) {
        GENZ_LOG_WARN(node_, "IMU stamps went backwards; cleared both streams, estimator will restart");
    } else if (event == ScanBuffer::Event::InvalidImu) {
        GENZ_LOG_WARN(node_, "non-finite IMU sample skipped");
    }
}

#ifdef GENZ_LIO_WITH_LIVOX
#if GENZ_LIO_ROS_VERSION == 2
void OdometryServer::onLivox(const livox_ros_driver2::msg::CustomMsg &msg) {
#else
void OdometryServer::onLivox(const livox_ros_driver::CustomMsg &msg) {
#endif
    if (diagnostics_) diagnostics_->record("cloud_received", toSeconds(msg.header.stamp), 0,
                                          msg.point_num);
    RawScan scan;
    scan.has_time_offsets = true;
    scan.points.reserve(msg.point_num);
    for (std::size_t i = 0; i < msg.point_num; ++i) {
        const auto &point = msg.points[i];
        // Keep only returns the sensor itself considers good.
        const bool good_return = (point.tag & 0x30) == 0x10 || (point.tag & 0x30) == 0x00;
        if (!good_return) continue;

        Point out;
        out.position = Vec3(point.x, point.y, point.z);
        // The PointCloud2 path drops non-finite returns; do the same here, since
        // the range filters downstream compare against NaN and let it through.
        if (!out.position.allFinite()) continue;
        out.intensity = point.reflectivity;
        out.time_offset = static_cast<float>(point.offset_time * 1e-9);
        scan.points.push_back(out);
    }
    if (scan.points.empty()) {
        if (diagnostics_) diagnostics_->record("cloud_empty", toSeconds(msg.header.stamp));
        return;
    }

    // ScanBuffer derives extrema from all accepted returns, including unsorted packets.
    const double stamp = toSeconds(msg.header.stamp) + time_offset_;
    const auto event = buffer_.push(std::move(scan), stamp, 0.0);
    if (diagnostics_) diagnostics_->record("cloud_buffered", stamp, 0, buffer_.pendingScans(),
                                          0, 0, static_cast<int>(event));
}
#endif

void OdometryServer::spinOnce() {
    ScanBuffer::BufferedScan scan;
    std::vector<ImuSample> imu;
    while (buffer_.pop(scan, imu)) {
        if (scan.epoch != pipeline_epoch_) {
            pipeline_->reset();
            gravity_transform_sent_ = false;
            terminal_status_.reset();
            path_.poses.clear();
            pcd_points_.clear();
            pcd_frames_ = 0;
            pipeline_epoch_ = scan.epoch;
        }
        const auto result =
            pipeline_->registerScan(scan.scan, scan.begin_time, scan.end_time, imu);
        if (diagnostics_) diagnostics_->record("scan_processed", scan.begin_time, scan.end_time,
            imu.size(), imu.empty() ? 0 : imu.front().timestamp,
            imu.empty() ? 0 : imu.back().timestamp, result.valid, result.update_rejected);
        if (!result.valid) continue;

        // Per-scan diagnostics, off unless asked for. Laid out the way the
        // reference implementation printed them, and written with printf rather
        // than the logger so the numbers are not buried under log prefixes.
        static const bool trace = std::getenv("GENZ_LIO_TRACE") != nullptr;
        if (trace && !output_.terminal_status_en) {
            std::printf("\n[Scale-aware adaptive voxelization]\n");
            std::printf("Median range: %.2f\n", result.median_range);
            std::printf("Scale Indicator: %.2f\n", result.scale_indicator);
            std::printf("Setpoint: %.2f\n", result.setpoint);
            std::printf("Adaptive voxel size: %.4f\n", result.leaf_size);
            std::printf("# of original scan points: %d\n", result.scan_points);
            std::printf("# of voxelized points: %d\n", result.voxelized_points);
            std::printf("# of mapping points: %d\n", result.mapping_points);
            std::printf("Error: %d\n",
                        static_cast<int>(result.setpoint) - result.voxelized_points);

            std::printf("[Hybrid-metric update]\n");
            std::printf("Plane matches: %d\n", result.plane_matches);
            std::printf("Point matches: %d\n", result.point_matches);
            std::printf("Map voxels: %zu\n", pipeline_->map().size());
            std::printf("Frame time: %.1f ms  [pre %.1f  deskew %.1f  voxelize %.1f  "
                        "cov %.1f  update %.1f  map %.1f]\n",
                        result.processing_time_ms, result.timing.preprocess,
                        result.timing.deskew, result.timing.voxelize,
                        result.timing.covariance, result.timing.update, result.timing.map);
            std::fflush(stdout);
        }

        if (result.update_rejected) {
            GENZ_LOG_WARN(node_,
                          "ESIKF update produced a non-finite state at t=%.3f; fell back to the "
                          "IMU prediction and skipped the map update",
                          result.timestamp);
        }
        publish(result, toRosTime(result.timestamp));
        if (output_.terminal_status_en)
            terminal_status_.print(result, config_.adaptive_voxelization.scale_threshold);
    }
}

void OdometryServer::publishPoints(const Publisher<PointCloud2> &publisher,
                                   const std::vector<Vec3> &points, const Time &stamp,
                                   const std::string &frame) const {
    if (publisher.subscriberCount() == 0) return;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(points.size());
    for (const auto &point : points) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(point.x());
        p.y = static_cast<float>(point.y());
        p.z = static_cast<float>(point.z());
        cloud.push_back(p);
    }

    PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    publisher.publish(msg);
}

void OdometryServer::publish(const GenZLIO::Result &result, const Time &stamp) {
    const Eigen::Quaterniond orientation(result.pose.rotation);

    Odometry odom;
    odom.header.frame_id = output_.odom_frame;
    odom.child_frame_id = output_.body_frame;
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = result.pose.translation.x();
    odom.pose.pose.position.y = result.pose.translation.y();
    odom.pose.pose.position.z = result.pose.translation.z();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();
    odom.twist.twist.linear.x = result.velocity.x();
    odom.twist.twist.linear.y = result.velocity.y();
    odom.twist.twist.linear.z = result.velocity.z();
    odom_pub_.publish(odom);
    if (diagnostics_) diagnostics_->record("odometry_published", result.timestamp);

    tf_broadcaster_->send(stamp, output_.odom_frame, output_.body_frame,
                          result.pose.translation.x(), result.pose.translation.y(),
                          result.pose.translation.z(), orientation.x(), orientation.y(),
                          orientation.z(), orientation.w());

    // The odometry frame is only gravity-aligned through this transform, which
    // the IMU initialization fixes once and for all.
    if (!gravity_transform_sent_) {
        const Eigen::Quaterniond gravity_rotation(pipeline_->gravityAlignment());
        static_tf_broadcaster_->send(stamp, output_.world_frame, output_.odom_frame,
                                    gravity_rotation.x(), gravity_rotation.y(),
                                    gravity_rotation.z(), gravity_rotation.w());
        gravity_transform_sent_ = true;
        if (diagnostics_) diagnostics_->record("world_tf_static_published", result.timestamp);
    }

    if (output_.path_en) {
        PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        path_.poses.push_back(pose);
        path_.header.stamp = stamp;
        path_pub_.publish(path_);
    }

    publishCloud(result, stamp);
    if (output_.scan_semantic_publish_en) publishSemanticCloud(stamp);
    if (output_.voxel_map_en && voxel_map_pub_.subscriberCount() > 0) {
        voxel_map_pub_.publish(buildVoxelMapMarkers(pipeline_->map(), output_.voxel_map_max_layer,
                                                    output_.odom_frame, stamp));
    }
    if (output_.pcd_save_en) accumulateForPcd(result);
}

void OdometryServer::publishCloud(const GenZLIO::Result &result, const Time &stamp) {
    if (!output_.scan_publish_en) return;
    if (cloud_world_pub_.subscriberCount() == 0 && !output_.scan_bodyframe_pub_en) return;

    const state_ikfom state = pipeline_->state();
    if (cloud_world_pub_.subscriberCount() > 0) {
        // The seed scan does not run adaptive voxelization. Downsample only
        // its display copy, without advancing the voxelization controller.
        PointCloud seed_display;
        if (!output_.dense_publish_en && result.voxelized.empty()) {
            seed_display = voxelDownsample(result.deskewed, config_.mapping.down_sample_size);
        }
        const PointCloud &cloud = output_.dense_publish_en ? result.deskewed
            : (result.voxelized.empty() ? seed_display : result.voxelized);
        publishPoints(cloud_world_pub_, toWorld(cloud, state), stamp, output_.odom_frame);
        if (diagnostics_) diagnostics_->record("world_cloud_published", result.timestamp, 0,
            cloud.size(), result.deskewed.size(), result.voxelized.size(),
            output_.dense_publish_en ? 1 : 0);
    }
    if (output_.scan_bodyframe_pub_en && cloud_body_pub_.subscriberCount() > 0) {
        std::vector<Vec3> body;
        body.reserve(result.deskewed.size());
        for (const auto &point : result.deskewed) {
            body.push_back(state.offset_R_L_I * point.position + state.offset_T_L_I);
        }
        publishPoints(cloud_body_pub_, body, stamp, output_.body_frame);
    }
}

void OdometryServer::publishSemanticCloud(const Time &stamp) {
    std::vector<Vec3> planar;
    planar.reserve(pipeline_->planeMatches().size());
    for (const auto &match : pipeline_->planeMatches()) planar.push_back(match.point_world);

    std::vector<Vec3> non_planar;
    non_planar.reserve(pipeline_->pointMatches().size());
    for (const auto &match : pipeline_->pointMatches()) non_planar.push_back(match.point_world);

    publishPoints(planar_pub_, planar, stamp, output_.odom_frame);
    publishPoints(non_planar_pub_, non_planar, stamp, output_.odom_frame);
}

void OdometryServer::accumulateForPcd(const GenZLIO::Result &result) {
    const auto world = toWorld(result.deskewed, pipeline_->state());
    pcd_points_.insert(pcd_points_.end(), world.begin(), world.end());
    ++pcd_frames_;
    if (output_.pcd_save_interval > 0 && pcd_frames_ >= output_.pcd_save_interval) flushPcd();
}

void OdometryServer::flushPcd() {
    if (!output_.pcd_save_en || pcd_points_.empty()) return;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(pcd_points_.size());
    for (const auto &point : pcd_points_) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(point.x());
        p.y = static_cast<float>(point.y());
        p.z = static_cast<float>(point.z());
        cloud.push_back(p);
    }

    const std::string path =
        output_.pcd_directory + "/scans_" + std::to_string(pcd_index_++) + ".pcd";
    if (pcl::io::savePCDFileBinary(path, cloud) == 0) {
        GENZ_LOG_INFO(node_, "saved %zu points to %s", cloud.size(), path.c_str());
    } else {
        GENZ_LOG_ERROR(node_, "could not write %s", path.c_str());
    }
    pcd_points_.clear();
    pcd_frames_ = 0;
}

}  // namespace ros_wrapper
}  // namespace genz_lio
