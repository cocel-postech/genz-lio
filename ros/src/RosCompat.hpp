// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// The thin layer that lets one wrapper serve ROS 1 and ROS 2.
//
// The two differ in how nodes, publishers, subscriptions, transforms, time and
// logging are spelled, but not in what the wrapper needs from them. Everything
// version-specific is confined here so the node itself reads the same either
// way; GENZ_LIO_ROS_VERSION selects which half is compiled.
#pragma once

#include <functional>
#include <memory>
#include <string>

#if GENZ_LIO_ROS_VERSION == 2

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#else

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#endif

namespace genz_lio {
namespace ros_wrapper {

#if GENZ_LIO_ROS_VERSION == 2

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Imu = sensor_msgs::msg::Imu;
using Odometry = nav_msgs::msg::Odometry;
using Path = nav_msgs::msg::Path;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using Time = rclcpp::Time;
using Node = rclcpp::Node;

inline Time toRosTime(const double seconds) { return rclcpp::Time(static_cast<int64_t>(seconds * 1e9)); }
inline double toSeconds(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

/// Publisher with the one property the wrapper actually asks about: whether
/// anyone is listening, so that expensive conversions can be skipped.
template <typename M>
class Publisher {
public:
    Publisher() = default;
    Publisher(Node *node, const std::string &topic, const int depth)
        : handle_(node->create_publisher<M>(topic, rclcpp::QoS(depth))) {}

    void publish(const M &message) const { if (handle_) handle_->publish(message); }
    std::size_t subscriberCount() const { return handle_ ? handle_->get_subscription_count() : 0; }

private:
    typename rclcpp::Publisher<M>::SharedPtr handle_;
};

template <typename M>
using Subscription = typename rclcpp::Subscription<M>::SharedPtr;

template <typename M, typename Callback>
Subscription<M> createSubscription(Node *node, const std::string &topic, const int depth,
                                   Callback &&callback) {
    return node->create_subscription<M>(topic, rclcpp::QoS(rclcpp::KeepLast(depth)),
                                        std::forward<Callback>(callback));
}

class TfBroadcaster {
public:
    explicit TfBroadcaster(Node *node) : impl_(std::make_unique<tf2_ros::TransformBroadcaster>(node)) {}

    void send(const Time &stamp, const std::string &parent, const std::string &child,
              const double tx, const double ty, const double tz, const double qx, const double qy,
              const double qz, const double qw) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = parent;
        transform.child_frame_id = child;
        transform.transform.translation.x = tx;
        transform.transform.translation.y = ty;
        transform.transform.translation.z = tz;
        transform.transform.rotation.x = qx;
        transform.transform.rotation.y = qy;
        transform.transform.rotation.z = qz;
        transform.transform.rotation.w = qw;
        impl_->sendTransform(transform);
    }

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> impl_;
};

#define GENZ_LOG_INFO(node, ...) RCLCPP_INFO((node)->get_logger(), __VA_ARGS__)
#define GENZ_LOG_WARN(node, ...) RCLCPP_WARN((node)->get_logger(), __VA_ARGS__)
#define GENZ_LOG_ERROR(node, ...) RCLCPP_ERROR((node)->get_logger(), __VA_ARGS__)

template <typename T>
T getParameter(Node *node, const std::string &name, const T &fallback) {
    return node->declare_parameter<T>(name, fallback);
}

#else  // ROS 1

using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using Imu = sensor_msgs::Imu;
using Odometry = nav_msgs::Odometry;
using Path = nav_msgs::Path;
using PoseStamped = geometry_msgs::PoseStamped;
using MarkerArray = visualization_msgs::MarkerArray;
using Marker = visualization_msgs::Marker;
using Time = ros::Time;
using Node = ros::NodeHandle;

inline Time toRosTime(const double seconds) { return ros::Time().fromSec(seconds); }
inline double toSeconds(const ros::Time &stamp) { return stamp.toSec(); }

template <typename M>
class Publisher {
public:
    Publisher() = default;
    Publisher(Node *node, const std::string &topic, const int depth)
        : handle_(node->advertise<M>(topic, depth)) {}

    void publish(const M &message) const { handle_.publish(message); }
    std::size_t subscriberCount() const { return handle_.getNumSubscribers(); }

private:
    ros::Publisher handle_;
};

template <typename M>
using Subscription = ros::Subscriber;

template <typename M, typename Callback>
Subscription<M> createSubscription(Node *node, const std::string &topic, const int depth,
                                   Callback &&callback) {
    return node->subscribe<M>(topic, depth, std::function<void(const boost::shared_ptr<const M> &)>(
                                                std::forward<Callback>(callback)));
}

class TfBroadcaster {
public:
    explicit TfBroadcaster(Node *) {}

    void send(const Time &stamp, const std::string &parent, const std::string &child,
              const double tx, const double ty, const double tz, const double qx, const double qy,
              const double qz, const double qw) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(tx, ty, tz));
        transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
        impl_.sendTransform(tf::StampedTransform(transform, stamp, parent, child));
    }

private:
    tf::TransformBroadcaster impl_;
};

#define GENZ_LOG_INFO(node, ...) ROS_INFO(__VA_ARGS__)
#define GENZ_LOG_WARN(node, ...) ROS_WARN(__VA_ARGS__)
#define GENZ_LOG_ERROR(node, ...) ROS_ERROR(__VA_ARGS__)

template <typename T>
T getParameter(Node *node, const std::string &name, const T &fallback) {
    T value;
    node->param<T>(name, value, fallback);
    return value;
}

#endif

/// The gravity-aligned world and odometry frame have a fixed relationship
/// for an initialized pipeline epoch. Publish it latched, without a TF cache
/// time window that can expire while an expensive RViz map is being rendered.
class StaticTfBroadcaster {
public:
    explicit StaticTfBroadcaster(Node *node) {
#if GENZ_LIO_ROS_VERSION == 2
        impl_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
#else
        (void)node;
        impl_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
#endif
    }

    void send(const Time &stamp, const std::string &parent, const std::string &child,
              const double qx, const double qy, const double qz, const double qw) {
#if GENZ_LIO_ROS_VERSION == 2
        geometry_msgs::msg::TransformStamped transform;
#else
        geometry_msgs::TransformStamped transform;
#endif
        transform.header.stamp = stamp;
        transform.header.frame_id = parent;
        transform.child_frame_id = child;
        transform.transform.rotation.x = qx;
        transform.transform.rotation.y = qy;
        transform.transform.rotation.z = qz;
        transform.transform.rotation.w = qw;
        impl_->sendTransform(transform);
    }

private:
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> impl_;
};

}  // namespace ros_wrapper
}  // namespace genz_lio
