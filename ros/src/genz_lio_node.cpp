// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// ROS 1 entry point.
#include "OdometryServer.hpp"

#include <ros/ros.h>

#include <memory>

int main(int argc, char **argv) {
    ros::init(argc, argv, "genz_lio");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string config_path, sensor_config_path;
    private_nh.param<std::string>("config_path", config_path, "");
    private_nh.param<std::string>("sensor_config_path", sensor_config_path, "");

    using genz_lio::ros_wrapper::OdometryServer;
    std::unique_ptr<OdometryServer> server;
    try {
        server = std::make_unique<OdometryServer>(&private_nh, config_path, sensor_config_path);
    } catch (const std::exception &error) {
        ROS_FATAL("%s", error.what());
        return 1;
    }

    ros::Subscriber cloud_sub;
    if (server->expectsLivoxCustomMsg()) {
#ifdef GENZ_LIO_WITH_LIVOX
        cloud_sub = nh.subscribe<livox_ros_driver::CustomMsg>(
            server->lidarTopic(), 200000,
            [&](const livox_ros_driver::CustomMsg::ConstPtr &msg) { server->onLivox(*msg); });
#else
        ROS_FATAL(
            "lidar_type is 'livox', which needs the livox_ros_driver CustomMsg, but GenZ-LIO was "
            "built without it. Install and source livox_ros_driver and rebuild, or publish the "
            "sensor as PointCloud2 and set lidar_type to 'livox_pcl'.");
        return 1;
#endif
    } else {
        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
            server->lidarTopic(), 200000,
            [&](const sensor_msgs::PointCloud2::ConstPtr &msg) { server->onCloud(*msg); });
    }
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
        server->imuTopic(), 200000,
        [&](const sensor_msgs::Imu::ConstPtr &msg) { server->onImu(*msg); });

    // Callbacks fill the buffers on their own threads; the main loop drains
    // them, so a slow scan cannot stall message reception.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(500);
    while (ros::ok()) {
        server->spinOnce();
        rate.sleep();
    }
    spinner.stop();
    return 0;
}
