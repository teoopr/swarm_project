from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def explore_group(robot_ns: str):
    explore_params = PathJoinSubstitution([
        FindPackageShare('stage_multi_slam'),
        'config',
        'explore_stage.yaml'
    ])

    return GroupAction([
        PushRosNamespace(robot_ns),

        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[explore_params],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    ])


def generate_launch_description():
    first_pair = [
        explore_group('robot_0'),
        explore_group('robot_1'),
    ]

    second_pair = [
        explore_group('robot_2'),
        explore_group('robot_3'),
    ]

    return LaunchDescription([
        *first_pair,
        TimerAction(
            period=20.0,
            actions=second_pair
        ),
    ])
