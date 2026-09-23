# This file is part of GenZ-LIO, released under the GNU GPL v2.
#
# Load one complete YAML. Relative paths are under the package config/
# directory; absolute paths are used directly. No sensor overlay is applied.
#
#   ros2 launch genz_lio odometry.launch.py config:=default/ouster.yaml
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_fastdds_profile(context):
    # The workspace hook covers separately launched drivers/bag players. This
    # fallback also covers launching this file without sourcing that hook.
    if (context.environment.get("FASTRTPS_DEFAULT_PROFILES_FILE") or
            context.environment.get("FASTDDS_DEFAULT_PROFILES_FILE")):
        return []
    return [SetEnvironmentVariable(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        PathJoinSubstitution([FindPackageShare("genz_lio"), "config", "dds", "fastdds_local.xml"]),
    )]


def generate_launch_description():
    share = FindPackageShare("genz_lio")

    arguments = [
        DeclareLaunchArgument(
            "config",
            default_value="default/velodyne.yaml",
            description="Complete YAML: package config/ relative path or absolute path",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("lidar_topic", default_value="",
                              description="Override YAML LiDAR topic; empty keeps YAML"),
        DeclareLaunchArgument("imu_topic", default_value="",
                              description="Override YAML IMU topic; empty keeps YAML"),
    ]

    genz_lio = Node(
        package="genz_lio",
        executable="genz_lio_node",
        name="genz_lio",
        output="screen",
        emulate_tty=sys.stdout.isatty(),
        parameters=[
            {
                # PathJoinSubstitution uses pathlib: an absolute final component
                # replaces the package prefix, while a relative one is appended.
                "config_path": PathJoinSubstitution([share, "config", LaunchConfiguration("config")]),
                "lidar_topic": LaunchConfiguration("lidar_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", PathJoinSubstitution([share, "rviz", "genz_lio_ros2.rviz"])],
    )

    return LaunchDescription(arguments + [OpaqueFunction(function=_default_fastdds_profile),
                                          genz_lio, rviz])
