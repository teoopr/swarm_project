#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def ekf_node(robot_ns, odom_topic, condition):
    return Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=robot_ns,
        name='ekf_filter_node',
        output='screen',
        condition=condition,
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 0.25,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': f'{robot_ns}/odom',
            'base_link_frame': f'{robot_ns}/base_link',
            'world_frame': f'{robot_ns}/odom',
            'odom0': odom_topic,
            'odom0_config': [
                False, False, False,
                False, False, False,
                True, False, False,
                False, False, True,
                False, False, False
            ],
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_queue_size': 10,
            'imu0': 'imu/data',
            'imu0_config': [
                False, False, False,
                False, False, False,
                False, False, False,
                False, False, True,
                False, False, False
            ],
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_queue_size': 10,
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
    )


def ukf_node(robot_ns, odom_topic, condition):
    return Node(
        package='robot_localization',
        executable='ukf_node',
        namespace=robot_ns,
        name='ukf_filter_node',
        output='screen',
        condition=condition,
        parameters=[{
            'frequency': 10.0,
            'sensor_timeout': 0.25,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': f'{robot_ns}/odom',
            'base_link_frame': f'{robot_ns}/base_link',
            'world_frame': f'{robot_ns}/odom',
            'odom0': odom_topic,
            'odom0_config': [
                False, False, False,
                False, False, False,
                True, False, False,
                False, False, True,
                False, False, False
            ],
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_queue_size': 10,
            'imu0': 'imu/data',
            'imu0_config': [
                False, False, False,
                False, False, False,
                False, False, False,
                False, False, True,
                False, False, False
            ],
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_queue_size': 10,
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
    )


def noisy_odom_node(robot_ns, condition, random_seed, robot_index):
    robot_seed = PythonExpression(["int('", random_seed, "') + ", str(robot_index), " if int('", random_seed, "') >= 0 else -1"])

    return Node(
        package='potential_fields',
        executable='noisy_odom_node',
        namespace=robot_ns,
        name='noisy_odom_node',
        output='screen',
        condition=condition,
        parameters=[{
            'odom_frame': f'{robot_ns}/odom',
            'base_link_frame': f'{robot_ns}/base_link',
            'publish_tf': False,
            'linear_scale': 1.02,
            'angular_scale': 0.98,
            'linear_noise_std': 0.005,
            'lateral_noise_std': 0.0,
            'angular_noise_std': 0.01,
            'pose_covariance_xy': 0.05,
            'pose_covariance_yaw': 0.10,
            'twist_covariance_linear': 0.000025,
            'twist_covariance_angular': 0.0001,
            'random_seed': ParameterValue(robot_seed, value_type=int),
        }],
    )


def imu_sim_node(robot_ns, random_seed, robot_index):
    robot_seed = PythonExpression(["int('", random_seed, "') + 1000 + ", str(robot_index), " if int('", random_seed, "') >= 0 else -1"])

    return Node(
        package='potential_fields',
        executable='imu_sim_node',
        namespace=robot_ns,
        name='imu_sim_node',
        output='screen',
        parameters=[{
            'imu_frame': f'{robot_ns}/base_link',
            'gyro_scale': 1.005,
            'gyro_bias': 0.0,
            'gyro_noise_std': 0.0005,
            'gyro_covariance': 0.000025,
            'random_seed': ParameterValue(robot_seed, value_type=int),
        }],
    )


def amcl_node(robot_ns, x, y, yaw, condition):
    return Node(
        package='nav2_amcl',
        executable='amcl',
        namespace=robot_ns,
        name='amcl',
        output='screen',
        condition=condition,
        parameters=[{
            'global_frame_id': 'map',
            'odom_frame_id': f'{robot_ns}/odom',
            'base_frame_id': f'{robot_ns}/base_link',
            'scan_topic': 'base_scan',
            'map_topic': '/map',
            'tf_broadcast': True,
            'set_initial_pose': True,
            'always_reset_initial_pose': True,
            'initial_pose': {
                'x': ParameterValue(x, value_type=float),
                'y': ParameterValue(y, value_type=float),
                'z': 0.0,
                'yaw': ParameterValue(yaw, value_type=float),
            },
            'min_particles': 1000,
            'max_particles': 4000,
            'pf_err': 0.03,
            'pf_z': 0.99,
            'resample_interval': 1,
            'update_min_d': 0.05,
            'update_min_a': 0.05,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'laser_model_type': 'likelihood_field_prob',
            'max_beams': 80,
            'do_beamskip': True,
            'beam_skip_distance': 0.5,
            'beam_skip_threshold': 0.3,
            'beam_skip_error_threshold': 0.9,
            'z_hit': 0.7,
            'z_rand': 0.3,
            'sigma_hit': 0.25,
            'laser_likelihood_max_dist': 2.0,
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
    )


def amcl_lifecycle_manager(robot_ns, condition):
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=robot_ns,
        name='lifecycle_manager_amcl',
        output='screen',
        condition=condition,
        parameters=[{
            'autostart': True,
            'node_names': ['amcl'],
            'bond_timeout': 15.0,
            'bond_heartbeat_period': 0.25,
        }],
    )


def map_to_odom_node(robot_ns, x, y, yaw, condition):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'map_to_{robot_ns}_odom',
        output='screen',
        condition=condition,
        arguments=[
            '--x', x,
            '--y', y,
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', yaw,
            '--frame-id', 'map',
            '--child-frame-id', f'{robot_ns}/odom',
        ],
    )


def base_to_laser_node(robot_ns):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{robot_ns}_base_to_laser',
        output='screen',
        arguments=[
            '--x', '0.15',
            '--y', '0',
            '--z', '0.22',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', f'{robot_ns}/base_link',
            '--child-frame-id', f'{robot_ns}/laser',
        ],
    )


def pf_controller_node(robot_ns, mode, pf_parametri, formacija, ukljuceno):
    return Node(
        package='potential_fields',
        executable='pf_controller',
        namespace=robot_ns,
        name='pf_controller',
        output='screen',
        parameters=[
            pf_parametri,
            {
                'mode': mode,
                'robot_base_frame': f'{robot_ns}/base_link',
                'formacija': ParameterValue(formacija, value_type=str),
                'ukljuceno': ukljuceno,
            }
        ]
    )


def localization_evaluator_node(condition, robot_names, lok_csv_path, lokalizacija, run_id, random_seed, cilj_tolerancija):
    return Node(
        package='potential_fields',
        executable='localization_evaluator',
        name='localization_evaluator',
        output='screen',
        condition=condition,
        parameters=[{
            'robot_names': robot_names,
            'lok_csv_path': ParameterValue(lok_csv_path, value_type=str),
            'run_id': ParameterValue(run_id, value_type=int),
            'metoda': ParameterValue(lokalizacija, value_type=str),
            'random_seed': ParameterValue(random_seed, value_type=int),
            'cilj_tolerancija': cilj_tolerancija,
        }],
    )


def navigation_evaluator_node(condition, pf_parametri, robot_names, nav_csv_path, lokalizacija, run_id, random_seed, formacija, cilj_tolerancija):
    return Node(
        package='potential_fields',
        executable='navigation_evaluator',
        name='navigation_evaluator',
        output='screen',
        condition=condition,
        parameters=[
            pf_parametri,
            {
                'robot_names': robot_names,
                'cilj_tolerancija': cilj_tolerancija,
                'follower_cilj_tolerancija': 0.35,
                'formacija': ParameterValue(formacija, value_type=str),
                'run_id': ParameterValue(run_id, value_type=int),
                'metoda': ParameterValue(lokalizacija, value_type=str),
                'random_seed': ParameterValue(random_seed, value_type=int),
                'nav_csv_path': ParameterValue(nav_csv_path, value_type=str),
            }
        ],
    )


def generate_launch_description():
    stage_pkg_dir = get_package_share_directory('stage_ros2')
    pf_pkg_dir = get_package_share_directory('potential_fields')

    pf_parametri = os.path.join(pf_pkg_dir, 'config', 'pf_parametri.yaml')
    formacija = LaunchConfiguration('formacija')
    stage_world = PathJoinSubstitution([stage_pkg_dir, 'world', PythonExpression(["'crta_four_robots_' + '", formacija, "' + '.world'"])])
    map_yaml = os.path.join(pf_pkg_dir, 'maps', 'crta.yaml')
    rviz_config = os.path.join(pf_pkg_dir, 'config', 'pf.rviz')

    lokalizacija = LaunchConfiguration('lokalizacija')
    sum_odometrije = LaunchConfiguration('sum_odometrije')
    lok_evaluacija = LaunchConfiguration('lok_evaluacija')
    nav_evaluacija = LaunchConfiguration('nav_evaluacija')
    lok_csv_path = LaunchConfiguration('lok_csv_path')
    nav_csv_path = LaunchConfiguration('nav_csv_path')
    run_id = LaunchConfiguration('run_id')
    random_seed = LaunchConfiguration('random_seed')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = PythonExpression(["'wheel/odom' if '", sum_odometrije, "' == 'true' else 'odom'"])

    ekf_condition = IfCondition(PythonExpression(["'", lokalizacija, "' == 'ekf_amcl' or '", lokalizacija, "' == 'ekf'"]))
    ukf_condition = IfCondition(PythonExpression(["'", lokalizacija, "' == 'ukf_amcl' or '", lokalizacija, "' == 'ukf'"]))
    amcl_condition = IfCondition(PythonExpression(["'", lokalizacija, "' == 'ekf_amcl' or '", lokalizacija, "' == 'ukf_amcl'"]))
    odom_condition = IfCondition(PythonExpression(["'", lokalizacija, "' == 'ekf' or '", lokalizacija, "' == 'ukf'"]))
    sum_odometrije_condition = IfCondition(sum_odometrije)
    lok_evaluacija_condition = IfCondition(lok_evaluacija)
    nav_evaluacija_condition = IfCondition(nav_evaluacija)

    with open(pf_parametri, 'r') as f:
        pf_yaml = yaml.safe_load(f)

    parametri = pf_yaml['/**']['ros__parameters']

    robot0_ukljuceno = parametri.get('robot0_ukljuceno', True)
    robot1_ukljuceno = parametri.get('robot1_ukljuceno', True)
    robot2_ukljuceno = parametri.get('robot2_ukljuceno', True)
    robot3_ukljuceno = parametri.get('robot3_ukljuceno', True)

    robot_names = [
        robot_name
        for robot_name, ukljuceno in [
            ('robot_0', robot0_ukljuceno),
            ('robot_1', robot1_ukljuceno),
            ('robot_2', robot2_ukljuceno),
            ('robot_3', robot3_ukljuceno),
        ]
        if ukljuceno
    ]

    cilj_tolerancija = float(parametri.get('cilj_tolerancija', 0.25))

    def spawn_vrijednost(robot_index, komponenta):
        return PythonExpression([
            str(parametri[f'romb_robot{robot_index}_spawn_{komponenta}']),
            " if '", formacija, "' == 'romb' else ",
            str(parametri[f'kvadrat_robot{robot_index}_spawn_{komponenta}']),
            " if '", formacija, "' == 'kvadrat' else ",
            str(parametri[f'linija_robot{robot_index}_spawn_{komponenta}'])
        ])

    spawn = {
        robot_index: {
            komponenta: spawn_vrijednost(robot_index, komponenta)
            for komponenta in ['x', 'y', 'yaw']
        }
        for robot_index in range(4)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'formacija', default_value='romb', choices=['romb', 'kvadrat', 'linija'], description='romb, kvadrat ili linija'
        ),

        DeclareLaunchArgument(
            'lokalizacija', default_value='ekf_amcl', choices=['ekf_amcl', 'ukf_amcl', 'ekf', 'ukf'],
            description='ekf_amcl, ukf_amcl, ekf ili ukf'
        ),

        DeclareLaunchArgument('sum_odometrije', default_value='true', choices=['true', 'false'], description='true ili false'),

        DeclareLaunchArgument(
            'lok_evaluacija', default_value='true', choices=['true', 'false'],
            description='Uključuje ili isključuje evaluator lokalizacije'
        ),

        DeclareLaunchArgument(
            'nav_evaluacija', default_value='false', choices=['true', 'false'],
            description='Uključuje ili isključuje evaluator navigacije'
        ),

        DeclareLaunchArgument(
            'lok_csv_path', default_value='/tmp/pf_localization_error.csv',
            description='Putanja do CSV datoteke za evaluaciju lokalizacije'
        ),

        DeclareLaunchArgument(
            'nav_csv_path', default_value='/tmp/pf_navigation_evaluation.csv',
            description='Putanja do CSV datoteke za evaluaciju održavanjaformacije'
        ),

        DeclareLaunchArgument('run_id', default_value='0', description='Redni broj ponavljanja pokusa'),

        DeclareLaunchArgument(
            'random_seed', default_value='-1', description='Seed za šum odometrije i IMU-a'
        ),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'], description='Korištenje simulacijskog vremena'
        ),

        SetParameter(name='use_sim_time', value=ParameterValue(use_sim_time, value_type=bool)),

        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{
                'use_stamped_velocity': False,
                'use_ackermann': False,
                'enforce_prefixes': False,
                'use_static_transformations': True,
                'one_tf_tree': True,
                'world_file': stage_world,
            }],
            remappings=[
                ('/tf', '/stage_tf'),
                ('/tf_static', '/stage_tf_static'),
            ],
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'frame_id': 'map',
            }]
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    output='screen',
                    parameters=[{
                        'autostart': True,
                        'node_names': ['map_server'],
                        'bond_timeout': 15.0,
                        'bond_heartbeat_period': 0.25,
                    }]
                ),

                base_to_laser_node('robot_0'),
                base_to_laser_node('robot_1'),
                base_to_laser_node('robot_2'),
                base_to_laser_node('robot_3'),
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                noisy_odom_node('robot_0', sum_odometrije_condition, random_seed, 0),
                noisy_odom_node('robot_1', sum_odometrije_condition, random_seed, 1),
                noisy_odom_node('robot_2', sum_odometrije_condition, random_seed, 2),
                noisy_odom_node('robot_3', sum_odometrije_condition, random_seed, 3),

                imu_sim_node('robot_0', random_seed, 0),
                imu_sim_node('robot_1', random_seed, 1),
                imu_sim_node('robot_2', random_seed, 2),
                imu_sim_node('robot_3', random_seed, 3),
            ]
        ),

        TimerAction(
            period=12.0,
            actions=[
                ekf_node('robot_0', odom_topic, ekf_condition),
                ekf_node('robot_1', odom_topic, ekf_condition),
                ekf_node('robot_2', odom_topic, ekf_condition),
                ekf_node('robot_3', odom_topic, ekf_condition),

                ukf_node('robot_0', odom_topic, ukf_condition),
                ukf_node('robot_1', odom_topic, ukf_condition),
                ukf_node('robot_2', odom_topic, ukf_condition),
                ukf_node('robot_3', odom_topic, ukf_condition),
            ]
        ),

        TimerAction(
            period=17.0,
            actions=[
                amcl_node('robot_0', spawn[0]['x'], spawn[0]['y'], spawn[0]['yaw'], amcl_condition),
                amcl_node('robot_1', spawn[1]['x'], spawn[1]['y'], spawn[1]['yaw'], amcl_condition),

                map_to_odom_node('robot_0', spawn[0]['x'], spawn[0]['y'], spawn[0]['yaw'], odom_condition),
                map_to_odom_node('robot_1', spawn[1]['x'], spawn[1]['y'], spawn[1]['yaw'], odom_condition),
            ]
        ),

        TimerAction(
            period=20.0,
            actions=[
                amcl_node('robot_2', spawn[2]['x'], spawn[2]['y'], spawn[2]['yaw'], amcl_condition),
                amcl_node('robot_3', spawn[3]['x'], spawn[3]['y'], spawn[3]['yaw'], amcl_condition),

                map_to_odom_node('robot_2', spawn[2]['x'], spawn[2]['y'], spawn[2]['yaw'], odom_condition),
                map_to_odom_node('robot_3', spawn[3]['x'], spawn[3]['y'], spawn[3]['yaw'], odom_condition),
            ]
        ),

        TimerAction(
            period=22.0,
            actions=[
                amcl_lifecycle_manager('robot_0', amcl_condition),
                amcl_lifecycle_manager('robot_1', amcl_condition),
            ]
        ),

        TimerAction(
            period=25.0,
            actions=[
                amcl_lifecycle_manager('robot_2', amcl_condition),
                amcl_lifecycle_manager('robot_3', amcl_condition),
            ]
        ),

        TimerAction(
            period=30.0,
            actions=[
                pf_controller_node('robot_0', 'leader', pf_parametri, formacija, robot0_ukljuceno),
                pf_controller_node('robot_1', 'follower', pf_parametri, formacija, robot1_ukljuceno),
                pf_controller_node('robot_2', 'follower', pf_parametri, formacija, robot2_ukljuceno),
                pf_controller_node('robot_3', 'follower', pf_parametri, formacija, robot3_ukljuceno),
            ]
        ),

        TimerAction(
            period=35.0,
            actions=[
                localization_evaluator_node(
                    lok_evaluacija_condition, robot_names, lok_csv_path, lokalizacija, run_id, random_seed, cilj_tolerancija
                ),

                navigation_evaluator_node(
                    nav_evaluacija_condition, pf_parametri, robot_names, nav_csv_path, lokalizacija,
                    run_id, random_seed, formacija, cilj_tolerancija
                ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config]
                ),
            ]
        ),
    ])