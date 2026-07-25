#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def ogranicenje(value, lo, hi):
    return max(lo, min(value, hi))

def normalizacija_kuta(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def yaw_iz_kvat(x, y, z, w):
    yaw = euler_from_quaternion([x, y, z, w])[2]
    return yaw

class PFController(Node):
    def __init__(self):
        super().__init__('pf_controller')

        self.declare_parameter('mode', 'leader')
        self.declare_parameter('scan_topic', 'base_scan')
        self.declare_parameter('cilj_topic', '/goal_pose')
        self.declare_parameter('leader_cilj_aktivno_topic', '/leader/cilj_aktivno')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'robot_0/base_link')
        self.declare_parameter('leader_base_frame', 'robot_0/base_link')

        self.declare_parameter('offset_x', -0.5)
        self.declare_parameter('offset_y', 0.5)

        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 0.25)
        self.declare_parameter('rep_udaljenost', 1.2)
        self.declare_parameter('k_v', 0.25)
        self.declare_parameter('k_w', 1.4)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('cilj_tolerancija', 0.25)
        self.declare_parameter('korak_zrake', 4)
        self.declare_parameter('ucestalost', 10.0)
        self.declare_parameter('ukljuceno', True)

        self.mode = self.get_parameter('mode').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cilj_topic = self.get_parameter('cilj_topic').value
        self.leader_cilj_aktivno_topic = self.get_parameter('leader_cilj_aktivno_topic').value

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.leader_base_frame = self.get_parameter('leader_base_frame').value

        self.offset_x = float(self.get_parameter('offset_x').value)
        self.offset_y = float(self.get_parameter('offset_y').value)

        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.rep_udaljenost = float(self.get_parameter('rep_udaljenost').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.k_w = float(self.get_parameter('k_w').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.cilj_tolerancija = float(self.get_parameter('cilj_tolerancija').value)
        self.korak_zrake = int(self.get_parameter('korak_zrake').value)
        self.ucestalost = float(self.get_parameter('ucestalost').value)
        self.ukljuceno = bool(self.get_parameter('ukljuceno').value)

        self.zadnji_scan = None
        self.cilj_x = None
        self.cilj_y = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ima_pose = False

        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_yaw = 0.0
        self.ima_pose_leadera = False

        self.ima_cilj = self.mode == 'follower'
        self.leader_cilj_aktivno = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        if self.mode == 'leader':
            self.create_subscription(PoseStamped, self.cilj_topic, self.cilj_callback, 10)
            self.cilj_aktivno_pub = self.create_publisher(Bool, self.leader_cilj_aktivno_topic, 10)
            self.cilj_aktivno_timer = self.create_timer(0.1, self.publish_cilj_aktivno)
        elif self.mode == 'follower':
            self.create_subscription(Bool, self.leader_cilj_aktivno_topic, self.leader_cilj_aktivno_callback, 10)
        else:
            raise ValueError("Parametar 'mode' mora biti 'leader' ili 'follower'.")

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / self.ucestalost, self.control_loop)

        self.get_logger().info("Controller se pokrenuo.")

    def scan_callback(self, msg):
        self.zadnji_scan = msg

    def azuriraj_svoju_pozu_iz_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time()
            )
        except TransformException:
            self.ima_pose = False
            return False

        self.x = tf.transform.translation.x
        self.y = tf.transform.translation.y
        q = tf.transform.rotation
        self.yaw = yaw_iz_kvat(q.x, q.y, q.z, q.w)
        self.ima_pose = True
        return True

    def azuriraj_pozu_leadera_iz_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.leader_base_frame,
                Time()
            )
        except TransformException:
            self.ima_pose_leadera = False
            return False

        self.leader_x = tf.transform.translation.x
        self.leader_y = tf.transform.translation.y
        q = tf.transform.rotation
        self.leader_yaw = yaw_iz_kvat(q.x, q.y, q.z, q.w)
        self.ima_pose_leadera = True
        self.ima_cilj = True
        return True

    def leader_cilj_aktivno_callback(self, msg):
        self.leader_cilj_aktivno = msg.data

    def cilj_callback(self, msg):
        self.cilj_x = msg.pose.position.x
        self.cilj_y = msg.pose.position.y
        self.ima_cilj = True
        self.leader_cilj_aktivno = True
        self.get_logger().info(f'Novi cilj: ({self.cilj_x:.2f}, {self.cilj_y:.2f})')

    def publish_cilj_aktivno(self):
        msg = Bool()
        msg.data = self.leader_cilj_aktivno
        self.cilj_aktivno_pub.publish(msg)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def dohvati_trenutni_cilj(self):
        if self.mode == 'leader':
            return self.cilj_x, self.cilj_y

        gx = self.leader_x + self.offset_x * math.cos(self.leader_yaw) - self.offset_y * math.sin(self.leader_yaw)
        gy = self.leader_y + self.offset_x * math.sin(self.leader_yaw) + self.offset_y * math.cos(self.leader_yaw)
        return gx, gy

    def izracun_privlacenje(self, gx, gy):
        return self.k_att * (gx - self.x), self.k_att * (gy - self.y)

    def izracun_odbijanje(self):
        if self.zadnji_scan is None:
            return 0.0, 0.0

        fx_rep = 0.0
        fy_rep = 0.0

        for i in range(0, len(self.zadnji_scan.ranges), self.korak_zrake):
            d = self.zadnji_scan.ranges[i]

            if math.isinf(d) or math.isnan(d):
                continue

            if d < self.zadnji_scan.range_min or d > self.rep_udaljenost:
                continue

            zraka_kut = self.zadnji_scan.angle_min + i * self.zadnji_scan.angle_increment
            iznos = self.k_rep * ((1.0 / d) - (1.0 / self.rep_udaljenost)) / (d * d)

            kut_prepreka = self.yaw + zraka_kut
            fx_rep += -iznos * math.cos(kut_prepreka)
            fy_rep += -iznos * math.sin(kut_prepreka)

        return fx_rep, fy_rep

    def control_loop(self):
        if not self.ukljuceno:
            self.stop_robot()
            return

        if self.zadnji_scan is None or not self.ima_cilj:
            return

        if not self.azuriraj_svoju_pozu_iz_tf():
            return

        if self.mode == 'follower':
            if not self.leader_cilj_aktivno:
                self.stop_robot()
                return
            if not self.azuriraj_pozu_leadera_iz_tf():
                return

        cilj_x, cilj_y = self.dohvati_trenutni_cilj()

        dx = cilj_x - self.x
        dy = cilj_y - self.y
        udaljenost_cilja = math.hypot(dx, dy)

        if udaljenost_cilja < self.cilj_tolerancija:
            self.stop_robot()
            return

        fx_att, fy_att = self.izracun_privlacenje(cilj_x, cilj_y)
        fx_rep, fy_rep = self.izracun_odbijanje()

        fx = fx_att + fx_rep
        fy = fy_att + fy_rep

        zeljeni_smjer = math.atan2(fy, fx)
        razlika_smjera = normalizacija_kuta(zeljeni_smjer - self.yaw)
        jacina_sile = math.hypot(fx, fy)

        linearna_x = self.k_v * jacina_sile * max(0.0, math.cos(razlika_smjera))
        kutna_z = self.k_w * razlika_smjera

        linearna_x = ogranicenje(linearna_x, 0.0, self.v_max)
        kutna_z = ogranicenje(kutna_z, -self.w_max, self.w_max)

        cmd = Twist()
        cmd.linear.x = linearna_x
        cmd.angular.z = kutna_z
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()