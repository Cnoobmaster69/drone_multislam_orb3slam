#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context):
    ns = LaunchConfiguration("robot_namespace").perform(context).strip()
    if ns == "":
        ns = "uav_1"

    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    base_link = f"{ns}/base_link"
    camera_link = f"{ns}/camera_link"
    camera_optical = f"{ns}/camera_optical_frame"

    # Physical extrinsics: base_link -> camera_link
    base_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{ns}_base_to_camera_link_tf",
        output="screen",
        arguments=[x, y, z, roll, pitch, yaw, base_link, camera_link],
    )

    # Optical convention: camera_link -> camera_optical_frame
    cam_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{ns}_camera_to_optical_tf",
        output="screen",
        arguments=[
            "0", "0", "0",
            "-1.57079632679", "0", "-1.57079632679",
            camera_link, camera_optical,
        ],
    )

    map_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{ns}_map_to_world_tf",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "map", "world",
        ],
    )

    return [base_to_cam, cam_to_optical, map_to_world]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="uav_1",
                description="Agent namespace, e.g. uav_1, uav_2",
            ),
            DeclareLaunchArgument("x", default_value="0.12", description="base->camera tx"),
            DeclareLaunchArgument("y", default_value="0.03", description="base->camera ty"),
            DeclareLaunchArgument("z", default_value="0.242", description="base->camera tz"),
            DeclareLaunchArgument("roll", default_value="0.0", description="base->camera roll"),
            DeclareLaunchArgument("pitch", default_value="0.0", description="base->camera pitch"),
            DeclareLaunchArgument("yaw", default_value="0.0", description="base->camera yaw"),
            OpaqueFunction(function=_build_nodes),
        ]
    )
