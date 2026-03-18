#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")
    gz_bridge_config = LaunchConfiguration("gz_bridge_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock for ros_gz_bridge",
    )

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="uav_1",
        description="Namespace for static TF launch",
    )

    declare_gz_bridge_config = DeclareLaunchArgument(
        "gz_bridge_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("multi_slam"), "config", "gz_bridge.yaml"]
        ),
        description="Path to ros_gz_bridge YAML config",
    )

    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "--ros-args",
            "-p",
            ["config_file:=", gz_bridge_config],
            "-p",
            ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
    )

    ros1_bridge = ExecuteProcess(
        cmd=["ros2", "run", "ros1_bridge", "parameter_bridge"],
        output="screen",
    )

    static_frames = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("multi_slam"), "launch", "static_frames.launch.py"]
            )
        ),
        launch_arguments={"robot_namespace": robot_namespace}.items(),
    )

    orb_slam3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orb_slam3_ros2_wrapper"), "launch", "rgbd.launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_robot_namespace,
            declare_gz_bridge_config,
            gz_bridge,
            ros1_bridge,
            static_frames,
            orb_slam3,
        ]
    )
