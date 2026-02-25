#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')
#---------------------------------------------

    # LAUNCH ARGS
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value="",
        description='The namespace of the robot')
    ag_n = LaunchConfiguration('ag_n')
    ag_n_arg = DeclareLaunchArgument(
        'ag_n',
        default_value='0',
        description='Agent index for multi-agent runs (used to auto-build namespace when needed)')
#---------------------------------------------

    def all_nodes_launch(context, robot_namespace, ag_n):
        params_file = LaunchConfiguration('params_file')
        vocabulary_file_path = "/home/carlos/ws_offboard_control/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        # config_file_path = "/home/carlos/ws_offboard_control/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/euroc_stereo.yaml"
        config_file_path = "/home/carlos/ws_offboard_control/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/gazebo_rgbd.yaml"
        # config_file_path = "/home/carlos/ws_offboard_control/src/multi_slam/config/rgbd.yaml"
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(orb_wrapper_pkg, 'params', 'ros_params', 'gazebo-rgbd-ros-params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        namespace_value = robot_namespace.perform(context).strip()
        agent_value = ag_n.perform(context).strip()
        try:
            ag_idx = int(agent_value)
        except ValueError:
            ag_idx = 0

        # Priority:
        # 1) explicit robot_namespace if provided
        # 2) fallback namespace derived from ag_n (ag_n=0 -> uav_1)
        if namespace_value == "":
            namespace_value = f"uav_{ag_idx + 1}"

        # Force per-agent input topics so each agent subscribes to its own camera streams.
        # Absolute topic names avoid accidental double namespacing.
        param_substitutions = {
            'rgb_image_topic_name': f'/{namespace_value}/rgb/image_raw',
            'depth_image_topic_name': f'/{namespace_value}/depth/image',
            'robot_base_frame': f'{namespace_value}/base_link',
            'odom_frame': f'{namespace_value}/odom',
        }


        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace_value,
            param_rewrites=param_substitutions,
            convert_types=True)
        
        orb_slam3_node = Node(
            package='orb_slam3_ros2_wrapper',
            executable='rgbd',
            output='screen',
            # prefix=["gdbserver localhost:3000"],
            namespace=namespace_value,
            arguments=[vocabulary_file_path, config_file_path],
            parameters=[configured_params])
        
        return [declare_params_file_cmd, orb_slam3_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch, args=[robot_namespace, ag_n])
#---------------------------------------------

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        ag_n_arg,
        opaque_function
    ])
