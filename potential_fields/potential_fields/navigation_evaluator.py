#!/usr/bin/env python3

import csv
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion


def yaw_iz_kvat(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class NavigationEvaluator(Node):
    def __init__(self):
        super().__init__('navigation_evaluator')

        self.declare_parameter('robot_names', ['robot_0', 'robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('leader_name', 'robot_0')
        self.declare_parameter('ground_truth_topic_suffix', 'ground_truth')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('uski_prostor_topic', '/leader/uski_prostor')
        self.declare_parameter('eval_frequency', 5.0)

        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('cilj_tolerancija', 0.25)
        self.declare_parameter('follower_cilj_tolerancija', 0.08)
        self.declare_parameter('arrival_hold_time', 1.0)
        self.declare_parameter('finish_delay', 2.0)
        self.declare_parameter('max_path_step', 1.0)

        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', False)

        self.declare_parameter('formacija', 'romb')

        for formacija, vrijednosti in {
            'romb': [(-1.0, -1.0), (-1.0, 1.0), (-2.0, 0.0)],
            'kvadrat': [(0.0, -1.0), (-1.0, 0.0), (-1.0, -1.0)],
            'linija': [(-1.0, 0.0), (-2.0, 0.0), (-3.0, 0.0)],
        }.items():
            for indeks, (offset_x, offset_y) in enumerate(vrijednosti, start=1):
                self.declare_parameter(f'{formacija}_robot{indeks}_offset_x', offset_x)
                self.declare_parameter(f'{formacija}_robot{indeks}_offset_y', offset_y)

        self.declare_parameter('run_id', 0)
        self.declare_parameter('metoda', '')
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('nav_csv_path', '/tmp/pf_navigation_evaluation.csv')

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.leader_name = str(self.get_parameter('leader_name').value)
        self.gt_suffix = str(self.get_parameter('ground_truth_topic_suffix').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.leader_base_frame = f'{self.leader_name}/base_link'
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.uski_prostor_topic = str(self.get_parameter('uski_prostor_topic').value)
        self.eval_frequency = float(self.get_parameter('eval_frequency').value)

        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.leader_tolerance = float(self.get_parameter('cilj_tolerancija').value)
        self.follower_tolerance = float(self.get_parameter('follower_cilj_tolerancija').value)
        self.arrival_hold_time = float(self.get_parameter('arrival_hold_time').value)
        self.finish_delay = float(self.get_parameter('finish_delay').value)
        self.max_path_step = float(self.get_parameter('max_path_step').value)

        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)

        self.odabrana_formacija = str(self.get_parameter('formacija').value).strip().lower()
        self.aktivna_formacija = self.odabrana_formacija
        self.offseti = self.ucitaj_offsete()

        self.run_id = int(self.get_parameter('run_id').value)
        self.metoda = str(self.get_parameter('metoda').value)
        self.random_seed = int(self.get_parameter('random_seed').value)
        self.nav_csv_path = str(self.get_parameter('nav_csv_path').value)
        korijen, nastavak = os.path.splitext(self.nav_csv_path)
        self.nav_summary_csv_path = f'{korijen}_summary{nastavak or ".csv"}'

        self.ground_truth = {}
        self.occupied_x = None
        self.occupied_y = None
        self.map_resolution = None

        self.cilj = None
        self.vrijeme_pocetka = None
        self.evaluacija_aktivna = False
        self.leader_arrival_absolute_time = None
        self.evaluation_end_absolute_time = None
        self.evaluation_end_reason = ''
        self.last_leader_tf_goal_error = math.nan
        self.last_status_log_time = None
        self.stats = {}
        self.reset_metrics()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(Bool, self.uski_prostor_topic, self.uski_prostor_callback, 10)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)

        self.gt_subs = []
        for robot in self.robot_names:
            topic = f'/{robot}/{self.gt_suffix}'
            self.gt_subs.append(self.create_subscription(Odometry, topic, lambda msg, ime=robot: self.ground_truth_callback(ime, msg), 10))

        self.otvori_csv()
        self.timer = self.create_timer(1.0 / self.eval_frequency, self.timer_callback)

        self.get_logger().info('Navigation evaluator se pokrenuo.')
        self.get_logger().info(f'Puni CSV: {self.nav_csv_path}')
        self.get_logger().info(f'Sažeti CSV: {self.nav_summary_csv_path}')
        self.get_logger().info(f'Čeka se cilj na {self.goal_topic}.')
        self.get_logger().info(
            f'Završetak se provjerava preko TF-a {self.global_frame} -> '
            f'{self.leader_base_frame}, tolerancija={self.leader_tolerance:.3f} m.'
        )

    def ucitaj_offsete(self):
        offseti = {}
        for formacija in ['romb', 'kvadrat', 'linija']:
            offseti[formacija] = {self.leader_name: (0.0, 0.0)}
            for indeks in range(1, 4):
                robot = f'robot_{indeks}'
                offseti[formacija][robot] = (
                    float(self.get_parameter(f'{formacija}_robot{indeks}_offset_x').value),
                    float(self.get_parameter(f'{formacija}_robot{indeks}_offset_y').value)
                )
        return offseti

    def reset_metrics(self):
        self.stats = {
            robot: {
                'error_sum': 0.0,
                'error_count': 0,
                'max_error': -math.inf,
                'path_length': 0.0,
                'previous_position': None,
                'min_robot_center': math.inf,
                'min_robot_clearance': math.inf,
                'min_obstacle_clearance': math.inf,
                'inside_since': None,
                'arrival_time': math.nan,
                'arrived': False,
                'last_x': math.nan,
                'last_y': math.nan,
                'last_yaw': math.nan,
                'last_target_x': math.nan,
                'last_target_y': math.nan,
                'last_error': math.nan,
                'last_robot_center': math.nan,
                'last_robot_clearance': math.nan,
                'last_obstacle_clearance': math.nan,
            }
            for robot in self.robot_names
        }

    def otvori_csv(self):
        for putanja in [self.nav_csv_path, self.nav_summary_csv_path]:
            direktorij = os.path.dirname(putanja)
            if direktorij:
                os.makedirs(direktorij, exist_ok=True)

        self.csv_file = open(self.nav_csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            't_rel',
            'run_id',
            'metoda',
            'random_seed',
            'robot',
            'aktivna_formacija',
            'x_gt',
            'y_gt',
            'yaw_gt',
            'target_x',
            'target_y',
            'position_error_m',
            'leader_tf_goal_error_m',
            'nearest_robot_center_distance_m',
            'nearest_robot_clearance_m',
            'nearest_obstacle_clearance_m',
            'path_length_m',
            'arrived',
            'arrival_time_s'
        ])
        self.csv_file.flush()

    def goal_callback(self, msg):
        sada = self.get_clock().now().nanoseconds / 1e9
        self.cilj = (float(msg.pose.position.x), float(msg.pose.position.y), yaw_iz_kvat(msg.pose.orientation))
        self.vrijeme_pocetka = sada
        self.evaluacija_aktivna = True
        self.leader_arrival_absolute_time = None
        self.evaluation_end_absolute_time = None
        self.evaluation_end_reason = ''
        self.last_leader_tf_goal_error = math.nan
        self.last_status_log_time = None
        self.reset_metrics()

        self.get_logger().info(f'Primljen cilj ({self.cilj[0]:.3f}, {self.cilj[1]:.3f}). Evaluacija počinje.')

    def uski_prostor_callback(self, msg):
        nova_formacija = 'linija' if msg.data else self.odabrana_formacija
        if nova_formacija != self.aktivna_formacija:
            self.aktivna_formacija = nova_formacija
            self.get_logger().info(f'Aktivna formacija evaluatora: {nova_formacija}.')

    def ground_truth_callback(self, robot, msg):
        self.ground_truth[robot] = msg

    def map_callback(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)

        if width <= 0 or height <= 0:
            return

        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        occupied = data >= self.occupancy_threshold

        if self.unknown_is_obstacle:
            occupied = np.logical_or(occupied, data < 0)

        rows, cols = np.nonzero(occupied)
        resolution = float(msg.info.resolution)

        local_x = (cols.astype(np.float64) + 0.5) * resolution
        local_y = (rows.astype(np.float64) + 0.5) * resolution

        origin = msg.info.origin
        origin_yaw = yaw_iz_kvat(origin.orientation)
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)

        self.occupied_x = origin.position.x + c * local_x - s * local_y
        self.occupied_y = origin.position.y + s * local_x + c * local_y
        self.map_resolution = resolution

    def dohvati_pozu(self, msg):
        return (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw_iz_kvat(msg.pose.pose.orientation))

    def dohvati_target(self, robot, leader_pose):
        if robot == self.leader_name:
            return self.cilj[0], self.cilj[1]

        offset_x, offset_y = self.offseti[self.aktivna_formacija].get(robot, (0.0, 0.0))
        leader_x, leader_y, leader_yaw = leader_pose

        target_x = (leader_x + offset_x * math.cos(leader_yaw) - offset_y * math.sin(leader_yaw))
        target_y = (leader_y + offset_x * math.sin(leader_yaw) + offset_y * math.cos(leader_yaw))
        return target_x, target_y

    def obstacle_clearance(self, x, y):
        if self.occupied_x is None or len(self.occupied_x) == 0:
            return math.nan

        dx = self.occupied_x - x
        dy = self.occupied_y - y
        center_distance = math.sqrt(float(np.min(dx * dx + dy * dy)))
        half_cell_diagonal = 0.5 * math.sqrt(2.0) * self.map_resolution

        return center_distance - half_cell_diagonal - self.robot_radius

    def azuriraj_put(self, robot, x, y):
        stat = self.stats[robot]
        previous = stat['previous_position']

        if previous is not None:
            step = math.hypot(x - previous[0], y - previous[1])
            if step <= self.max_path_step:
                stat['path_length'] += step
            else:
                self.get_logger().warn(f'Ignoriran TF/GT skok za {robot}: {step:.3f} m.')

        stat['previous_position'] = (x, y)

    def provjeri_dolazak_leadera_iz_tf(self, sada):
        stat = self.stats[self.leader_name]

        if stat['arrived']:
            return

        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame, self.leader_base_frame, Time())
        except TransformException:
            return

        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        error = math.hypot(self.cilj[0] - x, self.cilj[1] - y)
        self.last_leader_tf_goal_error = error

        if (self.last_status_log_time is None or sada - self.last_status_log_time >= 2.0):
            self.get_logger().info(f'Predvodnik: TF udaljenost do cilja={error:.3f} m, ' f'tolerancija={self.leader_tolerance:.3f} m.')
            self.last_status_log_time = sada

        if error > self.leader_tolerance:
            stat['inside_since'] = None
            return

        if stat['inside_since'] is None:
            stat['inside_since'] = sada
            return

        if sada - stat['inside_since'] >= self.arrival_hold_time:
            stat['arrived'] = True
            stat['arrival_time'] = sada - self.vrijeme_pocetka
            self.leader_arrival_absolute_time = sada

            self.get_logger().info(
                f'{self.leader_name} je stigao prema TF-u za '
                f'{stat["arrival_time"]:.3f} s. Evaluacija završava za '
                f'{self.finish_delay:.3f} s.'
            )

    def azuriraj_dolazak_followera(self, robot, error, sada):
        stat = self.stats[robot]

        if stat['arrived']:
            return

        if not self.stats[self.leader_name]['arrived']:
            stat['inside_since'] = None
            return

        if error > self.follower_tolerance:
            stat['inside_since'] = None
            return

        if stat['inside_since'] is None:
            stat['inside_since'] = sada
            return

        if sada - stat['inside_since'] >= self.arrival_hold_time:
            stat['arrived'] = True
            stat['arrival_time'] = sada - self.vrijeme_pocetka
            self.get_logger().info(f'{robot} je unutar tolerancije za {stat["arrival_time"]:.3f} s.')

    def timer_callback(self):
        if not self.evaluacija_aktivna or self.cilj is None:
            return

        poses = {robot: self.dohvati_pozu(msg) for robot, msg in self.ground_truth.items() if robot in self.robot_names}

        if self.leader_name not in poses:
            return

        sada = self.get_clock().now().nanoseconds / 1e9
        t_rel = sada - self.vrijeme_pocetka
        leader_pose = poses[self.leader_name]
        self.provjeri_dolazak_leadera_iz_tf(sada)

        nearest_center = {robot: math.nan for robot in poses}
        nearest_clearance = {robot: math.nan for robot in poses}
        robots = list(poses)

        for i, robot_i in enumerate(robots):
            x_i, y_i, _ = poses[robot_i]
            for robot_j in robots[i + 1:]:
                x_j, y_j, _ = poses[robot_j]
                center = math.hypot(x_i - x_j, y_i - y_j)
                clearance = center - 2.0 * self.robot_radius

                for robot in [robot_i, robot_j]:
                    if math.isnan(nearest_center[robot]) or center < nearest_center[robot]:
                        nearest_center[robot] = center
                        nearest_clearance[robot] = clearance

                    stat = self.stats[robot]
                    stat['min_robot_center'] = min(stat['min_robot_center'], center)
                    stat['min_robot_clearance'] = min(stat['min_robot_clearance'], clearance)

        for robot, (x, y, yaw) in poses.items():
            target_x, target_y = self.dohvati_target(robot, leader_pose)
            error = math.hypot(target_x - x, target_y - y)
            stat = self.stats[robot]

            stat['error_sum'] += error
            stat['error_count'] += 1
            stat['max_error'] = max(stat['max_error'], error)
            self.azuriraj_put(robot, x, y)

            obstacle = self.obstacle_clearance(x, y)
            if not math.isnan(obstacle):
                stat['min_obstacle_clearance'] = min(stat['min_obstacle_clearance'], obstacle)

            stat['last_x'] = x
            stat['last_y'] = y
            stat['last_yaw'] = yaw
            stat['last_target_x'] = target_x
            stat['last_target_y'] = target_y
            stat['last_error'] = error
            stat['last_robot_center'] = nearest_center[robot]
            stat['last_robot_clearance'] = nearest_clearance[robot]
            stat['last_obstacle_clearance'] = obstacle

            if robot != self.leader_name:
                self.azuriraj_dolazak_followera(robot, error, sada)

            self.csv_writer.writerow([
                f'{sada:.9f}',
                f'{t_rel:.9f}',
                self.run_id,
                self.metoda,
                self.random_seed,
                robot,
                self.aktivna_formacija,
                f'{x:.9f}',
                f'{y:.9f}',
                f'{yaw:.9f}',
                f'{target_x:.9f}',
                f'{target_y:.9f}',
                f'{error:.9f}',
                self.format_value(self.last_leader_tf_goal_error),
                self.format_value(nearest_center[robot]),
                self.format_value(nearest_clearance[robot]),
                self.format_value(obstacle),
                f'{stat["path_length"]:.9f}',
                int(stat['arrived']),
                self.format_value(stat['arrival_time'])
            ])

        self.csv_file.flush()

        if (self.leader_arrival_absolute_time is not None and sada - self.leader_arrival_absolute_time >= self.finish_delay):
            self.evaluacija_aktivna = False
            self.evaluation_end_absolute_time = sada
            self.evaluation_end_reason = 'leader_arrived_finish_delay_elapsed'
            self.zapisi_summary()
            self.get_logger().info(
                'Predvodnik je stigao i isteklo je dodatno vrijeme zapisivanja od '
                f'{self.finish_delay:.3f} s. Evaluacija je završena.'
            )

    def format_value(self, value):
        if value is None or not math.isfinite(value):
            return ''
        return f'{value:.9f}'

    def zapisi_summary(self):
        if self.vrijeme_pocetka is None:
            return

        sada = self.get_clock().now().nanoseconds / 1e9
        kraj = (self.evaluation_end_absolute_time if self.evaluation_end_absolute_time is not None else sada)
        trajanje = max(0.0, kraj - self.vrijeme_pocetka)
        razlog = self.evaluation_end_reason or 'manual_stop_or_node_shutdown'

        with open(self.nav_summary_csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'run_id',
                'metoda',
                'random_seed',
                'robot',
                'aktivna_formacija',
                'sample_count',
                'evaluation_duration_s',
                'mean_position_error_m',
                'max_position_error_m',
                'final_position_error_m',
                'leader_tf_goal_error_at_end_m',
                'final_x_gt',
                'final_y_gt',
                'final_yaw_gt',
                'final_target_x',
                'final_target_y',
                'final_robot_center_distance_m',
                'final_robot_clearance_m',
                'final_obstacle_clearance_m',
                'min_robot_center_distance_m',
                'min_robot_clearance_m',
                'min_obstacle_clearance_m',
                'arrival_time_s',
                'path_length_m',
                'arrived',
                'final_in_tolerance',
                'evaluation_end_reason'
            ])

            for robot in self.robot_names:
                stat = self.stats[robot]
                mean_error = (stat['error_sum'] / stat['error_count'] if stat['error_count'] > 0 else math.nan)
                max_error = (stat['max_error'] if stat['error_count'] > 0 else math.nan)
                tolerancija = (self.leader_tolerance if robot == self.leader_name else self.follower_tolerance)
                final_in_tolerance = (math.isfinite(stat['last_error']) and stat['last_error'] <= tolerancija)

                writer.writerow([
                    self.run_id,
                    self.metoda,
                    self.random_seed,
                    robot,
                    self.aktivna_formacija,
                    stat['error_count'],
                    self.format_value(trajanje),
                    self.format_value(mean_error),
                    self.format_value(max_error),
                    self.format_value(stat['last_error']),
                    self.format_value(self.last_leader_tf_goal_error),
                    self.format_value(stat['last_x']),
                    self.format_value(stat['last_y']),
                    self.format_value(stat['last_yaw']),
                    self.format_value(stat['last_target_x']),
                    self.format_value(stat['last_target_y']),
                    self.format_value(stat['last_robot_center']),
                    self.format_value(stat['last_robot_clearance']),
                    self.format_value(stat['last_obstacle_clearance']),
                    self.format_value(stat['min_robot_center']),
                    self.format_value(stat['min_robot_clearance']),
                    self.format_value(stat['min_obstacle_clearance']),
                    self.format_value(stat['arrival_time']),
                    self.format_value(stat['path_length']),
                    int(stat['arrived']),
                    int(final_in_tolerance),
                    razlog
                ])

    def destroy_node(self):
        try:
            self.zapisi_summary()
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
                self.csv_file = None
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationEvaluator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()