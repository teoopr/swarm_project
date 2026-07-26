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
        self.declare_parameter('uski_prostor_topic', '/leader/uski_prostor')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'robot_0/base_link')
        self.declare_parameter('leader_base_frame', 'robot_0/base_link')

        self.declare_parameter('formacija', 'romb')

        self.declare_parameter('romb_robot1_offset_x', -1.0)
        self.declare_parameter('romb_robot1_offset_y', -1.0)
        self.declare_parameter('romb_robot2_offset_x', -1.0)
        self.declare_parameter('romb_robot2_offset_y', 1.0)
        self.declare_parameter('romb_robot3_offset_x', -2.0)
        self.declare_parameter('romb_robot3_offset_y', 0.0)

        self.declare_parameter('kvadrat_robot1_offset_x', 0.0)
        self.declare_parameter('kvadrat_robot1_offset_y', -1.0)
        self.declare_parameter('kvadrat_robot2_offset_x', -1.0)
        self.declare_parameter('kvadrat_robot2_offset_y', 0.0)
        self.declare_parameter('kvadrat_robot3_offset_x', -1.0)
        self.declare_parameter('kvadrat_robot3_offset_y', -1.0)

        self.declare_parameter('linija_robot1_offset_x', -1.0)
        self.declare_parameter('linija_robot1_offset_y', 0.0)
        self.declare_parameter('linija_robot2_offset_x', -2.0)
        self.declare_parameter('linija_robot2_offset_y', 0.0)
        self.declare_parameter('linija_robot3_offset_x', -3.0)
        self.declare_parameter('linija_robot3_offset_y', 0.0)

        self.declare_parameter('k_att', 1.5)
        self.declare_parameter('k_rep', 0.25)
        self.declare_parameter('rep_udaljenost', 1.2)
        self.declare_parameter('k_v', 0.35)
        self.declare_parameter('k_w', 1.4)
        self.declare_parameter('v_max', 0.7)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('cilj_tolerancija', 0.25)

        self.declare_parameter('follower_k_att', 2.0)
        self.declare_parameter('follower_k_rep', 0.15)
        self.declare_parameter('follower_rep_udaljenost', 0.9)
        self.declare_parameter('follower_k_v', 0.45)
        self.declare_parameter('follower_k_w', 1.8)
        self.declare_parameter('follower_v_max', 0.8)
        self.declare_parameter('follower_w_max', 1.6)
        self.declare_parameter('follower_cilj_tolerancija', 0.08)

        self.declare_parameter('korak_zrake', 4)
        self.declare_parameter('ucestalost', 10.0)
        self.declare_parameter('ukljuceno', True)

        self.declare_parameter('detekcija_uski_prostor', True)
        self.declare_parameter('sirina_uski_prostor_ulaz', 2.2)
        self.declare_parameter('sirina_uski_prostor_izlaz', 2.8)
        self.declare_parameter('uski_prostor_maks_domet', 3.5)
        self.declare_parameter('uski_prostor_lijevo_kut_min_deg', 70.0)
        self.declare_parameter('uski_prostor_lijevo_kut_max_deg', 110.0)
        self.declare_parameter('uski_prostor_desno_kut_min_deg', -110.0)
        self.declare_parameter('uski_prostor_desno_kut_max_deg', -70.0)
        self.declare_parameter('uski_prostor_min_ocitanja_po_strani', 3)

        self.mode = self.get_parameter('mode').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cilj_topic = self.get_parameter('cilj_topic').value
        self.leader_cilj_aktivno_topic = self.get_parameter('leader_cilj_aktivno_topic').value
        self.uski_prostor_topic = self.get_parameter('uski_prostor_topic').value

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.leader_base_frame = self.get_parameter('leader_base_frame').value

        self.odabrana_formacija = str(self.get_parameter('formacija').value).strip().lower()
        self.aktivna_formacija = self.odabrana_formacija
        self.robot_ime = self.dohvati_ime_robota()

        self.postavi_offset_za_formaciju()

        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.rep_udaljenost = float(self.get_parameter('rep_udaljenost').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.k_w = float(self.get_parameter('k_w').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.cilj_tolerancija = float(self.get_parameter('cilj_tolerancija').value)

        self.follower_k_att = float(self.get_parameter('follower_k_att').value)
        self.follower_k_rep = float(self.get_parameter('follower_k_rep').value)
        self.follower_rep_udaljenost = float(self.get_parameter('follower_rep_udaljenost').value)
        self.follower_k_v = float(self.get_parameter('follower_k_v').value)
        self.follower_k_w = float(self.get_parameter('follower_k_w').value)
        self.follower_v_max = float(self.get_parameter('follower_v_max').value)
        self.follower_w_max = float(self.get_parameter('follower_w_max').value)
        self.follower_cilj_tolerancija = float(self.get_parameter('follower_cilj_tolerancija').value)

        if self.mode == 'follower':
            self.k_att = self.follower_k_att
            self.k_rep = self.follower_k_rep
            self.rep_udaljenost = self.follower_rep_udaljenost
            self.k_v = self.follower_k_v
            self.k_w = self.follower_k_w
            self.v_max = self.follower_v_max
            self.w_max = self.follower_w_max
            self.cilj_tolerancija = self.follower_cilj_tolerancija

        self.korak_zrake = int(self.get_parameter('korak_zrake').value)
        self.ucestalost = float(self.get_parameter('ucestalost').value)
        self.ukljuceno = bool(self.get_parameter('ukljuceno').value)

        self.detekcija_uski_prostor = bool(self.get_parameter('detekcija_uski_prostor').value)
        self.sirina_uski_prostor_ulaz = float(self.get_parameter('sirina_uski_prostor_ulaz').value)
        self.sirina_uski_prostor_izlaz = float(self.get_parameter('sirina_uski_prostor_izlaz').value)
        self.uski_prostor_maks_domet = float(self.get_parameter('uski_prostor_maks_domet').value)

        self.uski_prostor_lijevo_kut_min = math.radians(float(self.get_parameter('uski_prostor_lijevo_kut_min_deg').value))
        self.uski_prostor_lijevo_kut_max = math.radians(float(self.get_parameter('uski_prostor_lijevo_kut_max_deg').value))
        self.uski_prostor_desno_kut_min = math.radians(float(self.get_parameter('uski_prostor_desno_kut_min_deg').value))
        self.uski_prostor_desno_kut_max = math.radians(float(self.get_parameter('uski_prostor_desno_kut_max_deg').value))
        self.uski_prostor_min_ocitanja_po_strani = int(self.get_parameter('uski_prostor_min_ocitanja_po_strani').value)

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

        self.uski_prostor = False
        self.procijenjena_sirina_prostora = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        if self.mode == 'leader':
            self.create_subscription(PoseStamped, self.cilj_topic, self.cilj_callback, 10)

            self.cilj_aktivno_pub = self.create_publisher(Bool, self.leader_cilj_aktivno_topic, 10)
            self.cilj_aktivno_timer = self.create_timer(0.1, self.publish_cilj_aktivno)

            self.uski_prostor_pub = self.create_publisher(Bool, self.uski_prostor_topic, 10)
            self.uski_prostor_timer = self.create_timer(0.1, self.publish_uski_prostor)

        elif self.mode == 'follower':
            self.create_subscription(Bool, self.leader_cilj_aktivno_topic, self.leader_cilj_aktivno_callback, 10)
            self.create_subscription(Bool, self.uski_prostor_topic, self.uski_prostor_callback, 10)
        else:
            raise ValueError("Parametar 'mode' mora biti 'leader' ili 'follower'.")

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / self.ucestalost, self.control_loop)

        self.get_logger().info("Controller se pokrenuo.")

    def dohvati_ime_robota(self):
        dijelovi = self.robot_base_frame.split('/')
        for dio in dijelovi:
            if dio.startswith('robot_'):
                return dio

        namespace = self.get_namespace().strip('/')
        if namespace.startswith('robot_'):
            return namespace

        return ''

    def postavi_offset_za_formaciju(self):
        if self.mode != 'follower':
            return

        formacija = self.aktivna_formacija

        if formacija not in ['romb', 'kvadrat', 'linija']:
            raise ValueError(f"Nepoznata formacija '{formacija}'.")

        if self.robot_ime not in ['robot_1', 'robot_2', 'robot_3']:
            raise ValueError(f"Nepoznat follower robot '{self.robot_ime}'.")

        robot_param = self.robot_ime.replace('_', '')
        param_x = f'{formacija}_{robot_param}_offset_x'
        param_y = f'{formacija}_{robot_param}_offset_y'

        self.offset_x = float(self.get_parameter(param_x).value)
        self.offset_y = float(self.get_parameter(param_y).value)

    def scan_callback(self, msg):
        self.zadnji_scan = msg

        if self.mode == 'leader' and self.detekcija_uski_prostor:
            self.azuriraj_uski_prostor()

    def medijan(self, vrijednosti):
        if not vrijednosti:
            return None

        vrijednosti = sorted(vrijednosti)
        n = len(vrijednosti)
        sredina = n // 2

        if n % 2 == 1:
            return vrijednosti[sredina]

        return 0.5 * (vrijednosti[sredina - 1] + vrijednosti[sredina])

    def ocitanja_u_kutnom_rasponu(self, scan, kut_min, kut_max):
        if scan is None or scan.angle_increment == 0.0:
            return []

        if kut_min > kut_max:
            kut_min, kut_max = kut_max, kut_min

        i_min = int(math.floor((kut_min - scan.angle_min) / scan.angle_increment))
        i_max = int(math.ceil((kut_max - scan.angle_min) / scan.angle_increment))

        i_min = max(0, i_min)
        i_max = min(len(scan.ranges) - 1, i_max)

        if i_min > i_max:
            return []

        ocitanja = []
        for i in range(i_min, i_max + 1):
            d = scan.ranges[i]

            if math.isinf(d) or math.isnan(d):
                continue

            if d < scan.range_min or d > self.uski_prostor_maks_domet:
                continue

            ocitanja.append(d)

        return ocitanja

    def procijeni_sirinu_prostora(self):
        if self.zadnji_scan is None:
            return None

        lijevo = self.ocitanja_u_kutnom_rasponu(self.zadnji_scan, self.uski_prostor_lijevo_kut_min, self.uski_prostor_lijevo_kut_max)
        desno = self.ocitanja_u_kutnom_rasponu(self.zadnji_scan, self.uski_prostor_desno_kut_min, self.uski_prostor_desno_kut_max)

        if len(lijevo) < self.uski_prostor_min_ocitanja_po_strani:
            return None

        if len(desno) < self.uski_prostor_min_ocitanja_po_strani:
            return None

        lijevo_medijan = self.medijan(lijevo)
        desno_medijan = self.medijan(desno)

        if lijevo_medijan is None or desno_medijan is None:
            return None

        return lijevo_medijan + desno_medijan

    def azuriraj_uski_prostor(self):
        sirina = self.procijeni_sirinu_prostora()
        self.procijenjena_sirina_prostora = sirina

        prethodno = self.uski_prostor

        if sirina is None:
            novo_stanje = False
        elif self.uski_prostor:
            novo_stanje = sirina < self.sirina_uski_prostor_izlaz
        else:
            novo_stanje = sirina < self.sirina_uski_prostor_ulaz

        self.uski_prostor = novo_stanje

        if self.uski_prostor != prethodno:
            if sirina is None:
                self.get_logger().info(f"Uski prostor: {self.uski_prostor}")
            else:
                self.get_logger().info(f"Uski prostor: {self.uski_prostor}, " f"procijenjena sirina: {sirina:.2f} m")

    def publish_uski_prostor(self):
        msg = Bool()
        msg.data = self.uski_prostor
        self.uski_prostor_pub.publish(msg)

    def uski_prostor_callback(self, msg):
        if self.mode != 'follower':
            return

        prethodno = self.uski_prostor
        self.uski_prostor = msg.data

        if self.uski_prostor:
            nova_formacija = 'linija'
        else:
            nova_formacija = self.odabrana_formacija

        if nova_formacija != self.aktivna_formacija:
            self.aktivna_formacija = nova_formacija
            self.postavi_offset_za_formaciju()
            self.get_logger().info(f"Promjena formacije: {self.aktivna_formacija}, " f"offset=({self.offset_x:.2f}, {self.offset_y:.2f})")
        elif self.uski_prostor != prethodno:
            self.get_logger().info(f"Uski prostor: {self.uski_prostor}")

    def azuriraj_svoju_pozu_iz_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.robot_base_frame, Time())
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
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.leader_base_frame, Time())
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

        gx = (self.leader_x + self.offset_x * math.cos(self.leader_yaw) - self.offset_y * math.sin(self.leader_yaw))
        gy = (self.leader_y + self.offset_x * math.sin(self.leader_yaw) + self.offset_y * math.cos(self.leader_yaw))
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