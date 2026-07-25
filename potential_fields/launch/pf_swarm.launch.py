#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    stage_pkg_dir = get_package_share_directory('stage_ros2')
    pf_pkg_dir = get_package_share_directory('potential_fields')
    pf_parametri = os.path.join(pf_pkg_dir, 'config', 'pf_parametri.yaml')

    stage_launch = os.path.join(stage_pkg_dir, 'launch', 'stage.launch.py')
    map_yaml = os.path.join(pf_pkg_dir, 'maps', 'crta.yaml')
    rviz_config = os.path.join(pf_pkg_dir, 'config', 'pf.rviz')

    with open(pf_parametri, 'r') as f:
        pf_yaml = yaml.safe_load(f)

    parametri = pf_yaml['/**']['ros__parameters']

    robot0_ukljuceno = parametri.get('robot0_ukljuceno', True)
    robot1_ukljuceno = parametri.get('robot1_ukljuceno', True)
    robot2_ukljuceno = parametri.get('robot2_ukljuceno', True)
    robot3_ukljuceno = parametri.get('robot3_ukljuceno', True)

    robot0_spawn_x = str(parametri.get('robot0_spawn_x', 18.0))
    robot0_spawn_y = str(parametri.get('robot0_spawn_y', 6.0))
    robot0_spawn_yaw = str(parametri.get('robot0_spawn_yaw', 3.141593))

    robot1_spawn_x = str(parametri.get('robot1_spawn_x', 18.5))
    robot1_spawn_y = str(parametri.get('robot1_spawn_y', 6.5))
    robot1_spawn_yaw = str(parametri.get('robot1_spawn_yaw', 3.141593))

    robot2_spawn_x = str(parametri.get('robot2_spawn_x', 18.5))
    robot2_spawn_y = str(parametri.get('robot2_spawn_y', 5.5))
    robot2_spawn_yaw = str(parametri.get('robot2_spawn_yaw', 3.141593))

    robot3_spawn_x = str(parametri.get('robot3_spawn_x', 19.0))
    robot3_spawn_y = str(parametri.get('robot3_spawn_y', 6.0))
    robot3_spawn_yaw = str(parametri.get('robot3_spawn_yaw', 3.141593))

    robot1_offset_x = parametri.get('robot1_offset_x', -0.5)
    robot1_offset_y = parametri.get('robot1_offset_y', 0.5)

    robot2_offset_x = parametri.get('robot2_offset_x', -0.5)
    robot2_offset_y = parametri.get('robot2_offset_y', -0.5)

    robot3_offset_x = parametri.get('robot3_offset_x', -1.0)
    robot3_offset_y = parametri.get('robot3_offset_y', 0.0)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage_launch),
            launch_arguments={
                'world': 'crta_four_robots',
                'one_tf_tree': 'true',
            }.items()
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml},
                {'use_sim_time': True},
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot0_odom',
            output='screen',
            arguments=[
                '--x', robot0_spawn_x,
                '--y', robot0_spawn_y,
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', robot0_spawn_yaw,
                '--frame-id', 'map',
                '--child-frame-id', 'robot_0/odom',
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot1_odom',
            output='screen',
            arguments=[
                '--x', robot1_spawn_x,
                '--y', robot1_spawn_y,
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', robot1_spawn_yaw,
                '--frame-id', 'map',
                '--child-frame-id', 'robot_1/odom',
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot2_odom',
            output='screen',
            arguments=[
                '--x', robot2_spawn_x,
                '--y', robot2_spawn_y,
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', robot2_spawn_yaw,
                '--frame-id', 'map',
                '--child-frame-id', 'robot_2/odom',
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_robot3_odom',
            output='screen',
            arguments=[
                '--x', robot3_spawn_x,
                '--y', robot3_spawn_y,
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', robot3_spawn_yaw,
                '--frame-id', 'map',
                '--child-frame-id', 'robot_3/odom',
            ]
        ),

        Node(
            package='potential_fields',
            executable='pf_controller',
            namespace='robot_0',
            name='pf_controller',
            output='screen',
            parameters=[
                pf_parametri,
                {
                    'mode': 'leader',
                    'scan_topic': 'base_scan',
                    'pose_topic': 'odom',
                    'cilj_topic': '/goal_pose',
                    'leader_cilj_aktivno_topic': '/leader/cilj_aktivno',
                    'global_frame': 'map',
                    'robot_base_frame': 'robot_0/base_link',
                    'leader_base_frame': 'robot_0/base_link',
                    'ukljuceno': robot0_ukljuceno,
                }
            ]
        ),

        Node(
            package='potential_fields',
            executable='pf_controller',
            namespace='robot_1',
            name='pf_controller',
            output='screen',
            parameters=[
                pf_parametri,
                {
                    'mode': 'follower',
                    'scan_topic': 'base_scan',
                    'pose_topic': 'odom',
                    'leader_pose_topic': '/robot_0/odom',
                    'leader_cilj_aktivno_topic': '/leader/cilj_aktivno',
                    'global_frame': 'map',
                    'robot_base_frame': 'robot_1/base_link',
                    'leader_base_frame': 'robot_0/base_link',
                    'offset_x': robot1_offset_x,
                    'offset_y': robot1_offset_y,
                    'ukljuceno': robot1_ukljuceno,
                }
            ]
        ),

        Node(
            package='potential_fields',
            executable='pf_controller',
            namespace='robot_2',
            name='pf_controller',
            output='screen',
            parameters=[
                pf_parametri,
                {
                    'mode': 'follower',
                    'scan_topic': 'base_scan',
                    'pose_topic': 'odom',
                    'leader_pose_topic': '/robot_0/odom',
                    'leader_cilj_aktivno_topic': '/leader/cilj_aktivno',
                    'global_frame': 'map',
                    'robot_base_frame': 'robot_2/base_link',
                    'leader_base_frame': 'robot_0/base_link',
                    'offset_x': robot2_offset_x,
                    'offset_y': robot2_offset_y,
                    'ukljuceno': robot2_ukljuceno,
                }
            ]
        ),

        Node(
            package='potential_fields',
            executable='pf_controller',
            namespace='robot_3',
            name='pf_controller',
            output='screen',
            parameters=[
                pf_parametri,
                {
                    'mode': 'follower',
                    'scan_topic': 'base_scan',
                    'pose_topic': 'odom',
                    'leader_pose_topic': '/robot_0/odom',
                    'leader_cilj_aktivno_topic': '/leader/cilj_aktivno',
                    'global_frame': 'map',
                    'robot_base_frame': 'robot_3/base_link',
                    'leader_base_frame': 'robot_0/base_link',
                    'offset_x': robot3_offset_x,
                    'offset_y': robot3_offset_y,
                    'ukljuceno': robot3_ukljuceno,
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config]
        ),
    ])