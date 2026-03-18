#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("orb_slam3_map_generator")
    _ = os.path.join(pkg_share, "param", "map_generator_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")
    auto_trigger_global_cloud = LaunchConfiguration("auto_trigger_global_cloud")
    trigger_delay_sec = LaunchConfiguration("trigger_delay_sec")
    input_pointcloud_rate = LaunchConfiguration("input_pointcloud_rate")
    max_pose_cloud_time_diff_sec = LaunchConfiguration("max_pose_cloud_time_diff_sec")
    use_normalized_time_diff = LaunchConfiguration("use_normalized_time_diff")
    sync_max_interval_sec = LaunchConfiguration("sync_max_interval_sec")
    output_reliable_qos = LaunchConfiguration("output_reliable_qos")
    depth_pointcloud_topic = LaunchConfiguration("depth_pointcloud_topic")
    trigger_max_attempts = LaunchConfiguration("trigger_max_attempts")
    trigger_retry_delay_sec = LaunchConfiguration("trigger_retry_delay_sec")
    trigger_z_thresh_max = LaunchConfiguration("trigger_z_thresh_max")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        name="robot_namespace",
        default_value="uav_1",
        description="Namespace for one agent (e.g. uav_0, uav_1)",
    )

    declare_auto_trigger_cmd = DeclareLaunchArgument(
        name="auto_trigger_global_cloud",
        default_value="False",
        description="If true, calls trigger_global_cloud once after launch",
    )

    declare_trigger_delay_cmd = DeclareLaunchArgument(
        name="trigger_delay_sec",
        default_value="1.0",
        description="Delay before auto trigger service call (seconds)",
    )

    declare_input_rate_cmd = DeclareLaunchArgument(
        name="input_pointcloud_rate",
        default_value="10.0",
        description="Expected input pointcloud rate (Hz)",
    )

    declare_depth_pointcloud_topic_cmd = DeclareLaunchArgument(
        name="depth_pointcloud_topic",
        default_value="camera/colored_pointcloud",
        description="PointCloud2 input for stitcher (e.g. camera/colored_pointcloud or depth/points)",
    )

    declare_max_diff_cmd = DeclareLaunchArgument(
        name="max_pose_cloud_time_diff_sec",
        default_value="0.35",
        description="Max allowed time difference between map pose and pointcloud (seconds).",
    )

    declare_use_normalized_time_diff_cmd = DeclareLaunchArgument(
        name="use_normalized_time_diff",
        default_value="False",
        description="If true, use normalized pose-cloud time matching instead of absolute timestamp diff.",
    )

    declare_sync_max_interval_cmd = DeclareLaunchArgument(
        name="sync_max_interval_sec",
        default_value="1.0",
        description="Approximate sync max interval for RGB+Depth pairing in seconds.",
    )

    declare_output_reliable_qos_cmd = DeclareLaunchArgument(
        name="output_reliable_qos",
        default_value="True",
        description="Publish camera/colored_pointcloud with reliable QoS.",
    )

    declare_trigger_attempts_cmd = DeclareLaunchArgument(
        name="trigger_max_attempts",
        default_value="8",
        description="How many trigger retries to attempt when no valid cloud yet.",
    )

    declare_trigger_retry_cmd = DeclareLaunchArgument(
        name="trigger_retry_delay_sec",
        default_value="2.0",
        description="Seconds between trigger retries.",
    )

    declare_trigger_z_thresh_cmd = DeclareLaunchArgument(
        name="trigger_z_thresh_max",
        default_value="50.0",
        description="z threshold used by stitch service request; <=0 disables z filtering.",
    )

    # Explicit per-node params: more robust than relying on YAML node-name matching under namespace.
    colored_cloud_params = {
        "use_sim_time": use_sim_time,
        "rgb_image_topic": "rgb/image_raw",
        "depth_image_topic": "depth/image",
        "rgb_info_topic": "rgb/camera_info",
        "depth_info_topic": "depth/camera_info",
        "output_pointcloud_topic": "camera/colored_pointcloud",
        "output_frame_id": "camera_optical_frame",
        "sync_max_interval_sec": sync_max_interval_sec,
        "output_reliable_qos": output_reliable_qos,
    }

    stitcher_params = {
        "use_sim_time": use_sim_time,
        "depth_pointcloud_topic": depth_pointcloud_topic,
        "map_data_topic": "map_data",
        "output_global_pointcloud_topic": "global_pointcloud",
        "input_pointcloud_rate": input_pointcloud_rate,
        "max_pose_cloud_time_diff_sec": max_pose_cloud_time_diff_sec,
        "use_normalized_time_diff": use_normalized_time_diff,
        "input_reliable_qos": output_reliable_qos,
        "robot_base_frame": "base_link",
    }

    depth_image_to_pcl_node = Node(
        package="orb_slam3_map_generator",
        executable="depth_image_to_pcl",
        namespace=robot_namespace,
        output="screen",
        parameters=[colored_cloud_params],
    )

    stitch_pcl_node = Node(
        package="orb_slam3_map_generator",
        executable="pointcloud_stitcher",
        namespace=robot_namespace,
        output="screen",
        parameters=[stitcher_params],
    )

    trigger_node = Node(
        package="orb_slam3_map_generator",
        executable="stitch_pcl_client.py",
        namespace=robot_namespace,
        output="screen",
        parameters=[{
            "service_name": "trigger_global_cloud",
            "max_attempts": trigger_max_attempts,
            "retry_delay_sec": trigger_retry_delay_sec,
            "z_thresh_max": trigger_z_thresh_max,
        }],
        condition=IfCondition(auto_trigger_global_cloud),
    )

    delayed_trigger = TimerAction(
        period=trigger_delay_sec,
        actions=[trigger_node],
        condition=IfCondition(auto_trigger_global_cloud),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_robot_namespace_cmd,
            declare_auto_trigger_cmd,
            declare_trigger_delay_cmd,
            declare_input_rate_cmd,
            declare_depth_pointcloud_topic_cmd,
            declare_max_diff_cmd,
            declare_use_normalized_time_diff_cmd,
            declare_sync_max_interval_cmd,
            declare_output_reliable_qos_cmd,
            declare_trigger_attempts_cmd,
            declare_trigger_retry_cmd,
            declare_trigger_z_thresh_cmd,
            depth_image_to_pcl_node,
            stitch_pcl_node,
            delayed_trigger,
        ]
    )
