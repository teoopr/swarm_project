#!/usr/bin/env python3

import csv
import math
import os
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion


def normalizacija_kuta(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_iz_kvat(x, y, z, w):
    return euler_from_quaternion([x, y, z, w])[2]


class LocalizationEvaluator(Node):
    def __init__(self):
        super().__init__('localization_evaluator')

        self.declare_parameter('robot_names', ['robot_0', 'robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame_suffix', 'base_link')
        self.declare_parameter('ground_truth_topic_suffix', 'ground_truth')
        self.declare_parameter('tf_timeout', 0.1)
        self.declare_parameter('eval_frequency', 2.0)
        self.declare_parameter('evaluation_delay', 0.3)
        self.declare_parameter('lok_csv_path', '/tmp/pf_localization_error.csv')
        self.declare_parameter('ucestalost_ispisa', 2.0)
        self.declare_parameter('run_id', 0)
        self.declare_parameter('metoda', '')
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('leader_name', 'robot_0')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cilj_tolerancija', 0.25)
        self.declare_parameter('stop_delay', 2.0)

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame_suffix = self.get_parameter('base_frame_suffix').value
        self.ground_truth_topic_suffix = self.get_parameter('ground_truth_topic_suffix').value
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.eval_frequency = float(self.get_parameter('eval_frequency').value)
        self.evaluation_delay = float(self.get_parameter('evaluation_delay').value)
        self.lok_csv_path = self.get_parameter('lok_csv_path').value
        self.ucestalost_ispisa = float(self.get_parameter('ucestalost_ispisa').value)
        self.run_id = int(self.get_parameter('run_id').value)
        self.metoda = self.get_parameter('metoda').value
        self.random_seed = int(self.get_parameter('random_seed').value)
        self.leader_name = self.get_parameter('leader_name').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cilj_tolerancija = float(self.get_parameter('cilj_tolerancija').value)
        self.stop_delay = float(self.get_parameter('stop_delay').value)

        self.ground_truth = {}
        self.ground_truth_history = {robot_name: deque(maxlen=100) for robot_name in self.robot_names}
        self.subs = []
        self.pubs = {}

        self.cilj_x = None
        self.cilj_y = None
        self.vrijeme_pocetka_zapisa = None
        self.vrijeme_zavrsetka_zapisa = None
        self.zapisivanje_aktivno = False
        self.zapisivanje_zavrseno = False
        self.predvodnik_stigao = False
        self.cilj_ceka_pocetak = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        for robot_name in self.robot_names:
            topic = f'/{robot_name}/{self.ground_truth_topic_suffix}'
            self.subs.append(self.create_subscription(Odometry, topic, lambda msg, ime=robot_name: self.ground_truth_callback(ime, msg), 10))
            self.pubs[robot_name] = self.create_publisher(Float64MultiArray, f'/{robot_name}/localization_error', 10)

        csv_dir = os.path.dirname(self.lok_csv_path)

        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        self.csv_file = open(self.lok_csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            't_rel',
            'run_id',
            'metoda',
            'random_seed',
            'robot',
            'xy_error',
            'xy_error_sq',
            'yaw_error',
            'yaw_abs_error',
            'yaw_error_sq',
            'yaw_error_deg',
            'yaw_abs_error_deg',
            'dx',
            'dy',
            'x_est',
            'y_est',
            'yaw_est',
            'x_gt',
            'y_gt',
            'yaw_gt'
        ])

        self.zadnji_ispis = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.eval_frequency, self.timer_callback)

        self.get_logger().info("Localization evaluator se pokrenuo.")
        self.get_logger().info("Čeka se cilj za početak zapisivanja.")

    def goal_callback(self, msg):
        novi_cilj_x = msg.pose.position.x
        novi_cilj_y = msg.pose.position.y

        if ((self.cilj_ceka_pocetak or self.zapisivanje_aktivno) and self.cilj_x is not None and self.cilj_y is not None):
            razlika_cilja = math.hypot(novi_cilj_x - self.cilj_x, novi_cilj_y - self.cilj_y)

            if razlika_cilja < 0.001:
                self.get_logger().info('Ponovljena poruka istog cilja je ignorirana.')
                return

        self.cilj_x = novi_cilj_x
        self.cilj_y = novi_cilj_y
        self.vrijeme_pocetka_zapisa = None
        self.vrijeme_zavrsetka_zapisa = None
        self.zapisivanje_aktivno = False
        self.zapisivanje_zavrseno = False
        self.predvodnik_stigao = False
        self.cilj_ceka_pocetak = True

        self.get_logger().info(
            f'Primljen cilj. Čeka se dostupnost svih robota i TF transformacija. '
            f'Cilj: x={self.cilj_x:.3f}, y={self.cilj_y:.3f}'
        )

    def ground_truth_callback(self, robot_name, msg):
        self.ground_truth[robot_name] = msg
        self.ground_truth_history[robot_name].append(msg)

    def dohvati_procjenu_iz_tf(self, robot_name, stamp=None):
        source_frame = f'{robot_name}/{self.base_frame_suffix}'

        if stamp is None:
            vrijeme = Time()
        else:
            vrijeme = Time.from_msg(stamp)

        tf = self.tf_buffer.lookup_transform(self.global_frame, source_frame, vrijeme, timeout=Duration(seconds=self.tf_timeout))

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_iz_kvat(q.x, q.y, q.z, q.w)

        return x, y, yaw

    def svi_roboti_spremni(self):
        for robot_name in self.robot_names:
            if robot_name not in self.ground_truth:
                return False

            try:
                self.dohvati_procjenu_iz_tf(robot_name)
            except TransformException:
                return False

        return True

    def dohvati_ground_truth(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_iz_kvat(q.x, q.y, q.z, q.w)

        return x, y, yaw

    def dohvati_ground_truth_za_vrijeme(self, robot_name, trazeno_vrijeme_sec):
        for msg in reversed(self.ground_truth_history[robot_name]):
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

            if stamp_sec <= trazeno_vrijeme_sec:
                return msg

        return None

    def provjeri_predvodnika_na_cilju(self, sada_sec):
        if self.predvodnik_stigao:
            return

        if self.cilj_x is None or self.cilj_y is None:
            return

        try:
            x, y, _ = self.dohvati_procjenu_iz_tf(self.leader_name)
        except TransformException:
            return

        udaljenost_do_cilja = math.hypot(self.cilj_x - x, self.cilj_y - y)

        if udaljenost_do_cilja <= self.cilj_tolerancija:
            self.predvodnik_stigao = True
            self.vrijeme_zavrsetka_zapisa = sada_sec + self.stop_delay
            self.get_logger().info(f'{self.leader_name} je stigao do cilja. Zapisivanje završava za {self.stop_delay:.1f} s.')

    def timer_callback(self):
        sada = self.get_clock().now()
        sada_sec = sada.nanoseconds / 1e9

        if self.cilj_ceka_pocetak:
            if not self.svi_roboti_spremni():
                return

            self.vrijeme_pocetka_zapisa = sada_sec
            self.vrijeme_zavrsetka_zapisa = None
            self.zapisivanje_aktivno = True
            self.zapisivanje_zavrseno = False
            self.predvodnik_stigao = False
            self.cilj_ceka_pocetak = False
            self.zadnji_ispis = sada

            self.get_logger().info('Svi roboti i TF transformacije su dostupni. Zapisivanje počinje.')

        if not self.zapisivanje_aktivno:
            return

        if self.zapisivanje_zavrseno:
            return

        if self.vrijeme_zavrsetka_zapisa is not None and sada_sec > self.vrijeme_zavrsetka_zapisa:
            self.zapisivanje_aktivno = False
            self.zapisivanje_zavrseno = True

            if self.csv_file is not None:
                self.csv_file.flush()

            self.get_logger().info("Zapisivanje lokalizacijske pogreške je završeno.")
            return

        trazeno_vrijeme_sec = sada_sec - self.evaluation_delay
        treba_ispis = (sada - self.zadnji_ispis).nanoseconds > int(self.ucestalost_ispisa * 1e9)

        for robot_name in self.robot_names:
            ground_truth_msg = self.dohvati_ground_truth_za_vrijeme(robot_name, trazeno_vrijeme_sec)

            if ground_truth_msg is None:
                continue

            vrijeme_uzorka_sec = float(ground_truth_msg.header.stamp.sec) + float(ground_truth_msg.header.stamp.nanosec) / 1e9
            t_rel = vrijeme_uzorka_sec - self.vrijeme_pocetka_zapisa

            if t_rel < 0.0:
                continue

            try:
                x_est, y_est, yaw_est = self.dohvati_procjenu_iz_tf(robot_name, ground_truth_msg.header.stamp)
            except TransformException:
                continue

            x_gt, y_gt, yaw_gt = self.dohvati_ground_truth(ground_truth_msg)

            dx = x_est - x_gt
            dy = y_est - y_gt
            xy_error = math.hypot(dx, dy)
            xy_error_sq = xy_error ** 2

            yaw_error = normalizacija_kuta(yaw_est - yaw_gt)
            yaw_abs_error = abs(yaw_error)
            yaw_error_sq = yaw_error ** 2
            yaw_error_deg = math.degrees(yaw_error)
            yaw_abs_error_deg = abs(yaw_error_deg)

            msg = Float64MultiArray()
            msg.data = [xy_error, yaw_error, yaw_abs_error, dx, dy, x_est, y_est, yaw_est, x_gt, y_gt, yaw_gt]
            self.pubs[robot_name].publish(msg)

            if self.csv_writer is not None:
                self.csv_writer.writerow([
                    vrijeme_uzorka_sec,
                    t_rel,
                    self.run_id,
                    self.metoda,
                    self.random_seed,
                    robot_name,
                    xy_error,
                    xy_error_sq,
                    yaw_error,
                    yaw_abs_error,
                    yaw_error_sq,
                    yaw_error_deg,
                    yaw_abs_error_deg,
                    dx,
                    dy,
                    x_est,
                    y_est,
                    yaw_est,
                    x_gt,
                    y_gt,
                    yaw_gt
                ])

            if treba_ispis:
                self.get_logger().info(
                    f'{robot_name}: xy_error={xy_error:.3f} m, '
                    f'yaw_abs_error={yaw_abs_error_deg:.2f} deg'
                )

        if self.csv_file is not None:
            self.csv_file.flush()

        self.provjeri_predvodnika_na_cilju(sada_sec)

        if treba_ispis:
            self.zadnji_ispis = sada

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEvaluator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()