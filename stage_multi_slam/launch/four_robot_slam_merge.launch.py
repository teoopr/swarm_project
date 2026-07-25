from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def slam_group(robot_ns: str):
    slam_params = PathJoinSubstitution([
        FindPackageShare('stage_multi_slam'),
        'config',
        'slam_toolbox_stage.yaml'
    ])

    return GroupAction([
        PushRosNamespace(robot_ns),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {
                    'use_sim_time': True,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'map_frame': 'map',
                    'scan_topic': 'base_scan',
                }
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/scan', 'base_scan'),
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
            ],
        ),
    ])


def generate_launch_description():
    map_merge_params = PathJoinSubstitution([
        FindPackageShare('stage_multi_slam'),
        'config',
        'map_merge_four_robots.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('multirobot_map_merge'),
        'launch',
        'map_merge.rviz'
    ])

    return LaunchDescription([
        slam_group('robot_0'),
        slam_group('robot_1'),
        slam_group('robot_2'),
        slam_group('robot_3'),

        Node(
            package='multirobot_map_merge',
            executable='map_merge',
            name='map_merge',
            output='screen',
            parameters=[map_merge_params],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen',
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_map_merge',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])