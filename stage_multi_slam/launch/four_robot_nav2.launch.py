from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def nav2_group(robot_ns: str):
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('stage_multi_slam'),
        'config',
        'nav2_stage.yaml'
    ])

    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=robot_ns,
        param_rewrites={},
        convert_types=True,
    )

    common_remaps = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel'),
        ('/odom', 'odom'),
        ('/scan', 'base_scan'),
        ('/map', 'map'),
        ('/map_updates', 'map_updates'),
    ]

    return GroupAction([
        PushRosNamespace(robot_ns),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'bond_timeout': 30.0},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]},
            ],
        ),
    ])


def generate_launch_description():
    first_pair = [
        nav2_group('robot_0'),
        nav2_group('robot_1'),
    ]

    second_pair = [
        nav2_group('robot_2'),
        nav2_group('robot_3'),
    ]

    return LaunchDescription([
        *first_pair,
        TimerAction(
            period=35.0,
            actions=second_pair
        ),
    ])