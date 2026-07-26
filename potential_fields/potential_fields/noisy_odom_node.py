#!/usr/bin/env python3

import copy
import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def normalizacija_kuta(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_iz_kvat(x, y, z, w):
    return euler_from_quaternion([x, y, z, w])[2]


class NoisyOdomNode(Node):
    def __init__(self):
        super().__init__('noisy_odom_node')

        self.declare_parameter('input_odom_topic', 'odom')
        self.declare_parameter('output_odom_topic', 'wheel/odom')

        self.declare_parameter('odom_frame', '')
        self.declare_parameter('base_link_frame', '')
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('linear_scale', 1.05)
        self.declare_parameter('angular_scale', 0.95)

        self.declare_parameter('linear_noise_std', 0.015)
        self.declare_parameter('lateral_noise_std', 0.004)
        self.declare_parameter('angular_noise_std', 0.03)

        self.declare_parameter('pose_covariance_xy', 0.05)
        self.declare_parameter('pose_covariance_yaw', 0.10)
        self.declare_parameter('twist_covariance_linear', 0.05)
        self.declare_parameter('twist_covariance_lateral', 0.05)
        self.declare_parameter('twist_covariance_angular', 0.10)

        self.declare_parameter('random_seed', -1)
        self.declare_parameter('motion_deadband', 0.000001)
        self.declare_parameter('angular_deadband', 0.000001)
        self.declare_parameter('velocity_deadband', 0.0001)
        self.declare_parameter('angular_velocity_deadband', 0.0001)

        self.input_odom_topic = (
            self.get_parameter('input_odom_topic').value
        )
        self.output_odom_topic = (
            self.get_parameter('output_odom_topic').value
        )

        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )
        self.base_link_frame = str(
            self.get_parameter('base_link_frame').value
        )
        self.publish_tf = bool(
            self.get_parameter('publish_tf').value
        )

        self.linear_scale = float(
            self.get_parameter('linear_scale').value
        )
        self.angular_scale = float(
            self.get_parameter('angular_scale').value
        )

        self.linear_noise_std = float(
            self.get_parameter('linear_noise_std').value
        )
        self.lateral_noise_std = float(
            self.get_parameter('lateral_noise_std').value
        )
        self.angular_noise_std = float(
            self.get_parameter('angular_noise_std').value
        )

        self.pose_covariance_xy = float(
            self.get_parameter('pose_covariance_xy').value
        )
        self.pose_covariance_yaw = float(
            self.get_parameter('pose_covariance_yaw').value
        )

        self.twist_covariance_linear = float(
            self.get_parameter('twist_covariance_linear').value
        )
        self.twist_covariance_lateral = float(
            self.get_parameter('twist_covariance_lateral').value
        )
        self.twist_covariance_angular = float(
            self.get_parameter('twist_covariance_angular').value
        )

        self.random_seed = int(
            self.get_parameter('random_seed').value
        )
        self.motion_deadband = float(
            self.get_parameter('motion_deadband').value
        )
        self.angular_deadband = float(
            self.get_parameter('angular_deadband').value
        )
        self.velocity_deadband = float(
            self.get_parameter('velocity_deadband').value
        )
        self.angular_velocity_deadband = float(
            self.get_parameter('angular_velocity_deadband').value
        )

        if self.random_seed >= 0:
            self.generator = random.Random(self.random_seed)
        else:
            self.generator = random.Random()

        self.zadnji_x = None
        self.zadnji_y = None
        self.zadnji_yaw = None
        self.zadnji_stamp = None

        self.noisy_x = 0.0
        self.noisy_y = 0.0
        self.noisy_yaw = 0.0

        self.odom_pub = self.create_publisher(
            Odometry,
            self.output_odom_topic,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'Noisy odom node pokrenut: '
            f'{self.input_odom_topic} -> {self.output_odom_topic}, '
            f'seed={self.random_seed}'
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = yaw_iz_kvat(q.x, q.y, q.z, q.w)

        stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) / 1e9
        )

        if self.zadnji_x is None:
            self.zadnji_x = x
            self.zadnji_y = y
            self.zadnji_yaw = yaw
            self.zadnji_stamp = stamp

            self.noisy_x = x
            self.noisy_y = y
            self.noisy_yaw = yaw

            self.objavi_odom(msg, 0.0, 0.0, 0.0)
            return

        dt = stamp - self.zadnji_stamp

        if dt <= 0.0:
            self.zadnji_x = x
            self.zadnji_y = y
            self.zadnji_yaw = yaw
            self.zadnji_stamp = stamp
            return

        dx = x - self.zadnji_x
        dy = y - self.zadnji_y
        dyaw = normalizacija_kuta(yaw - self.zadnji_yaw)

        dx_local = (
            math.cos(self.zadnji_yaw) * dx
            + math.sin(self.zadnji_yaw) * dy
        )

        dy_local = (
            -math.sin(self.zadnji_yaw) * dx
            + math.cos(self.zadnji_yaw) * dy
        )

        linearna_brzina = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )

        linearno_kretanje = (
            math.hypot(dx_local, dy_local) > self.motion_deadband
            or linearna_brzina > self.velocity_deadband
        )

        kutno_kretanje = (
            abs(dyaw) > self.angular_deadband
            or abs(msg.twist.twist.angular.z)
            > self.angular_velocity_deadband
        )

        if linearno_kretanje:
            noisy_dx_local = (
                self.linear_scale * dx_local
                + self.generator.gauss(
                    0.0,
                    self.linear_noise_std
                ) * dt
            )

            noisy_dy_local = (
                self.linear_scale * dy_local
                + self.generator.gauss(
                    0.0,
                    self.lateral_noise_std
                ) * dt
            )
        else:
            noisy_dx_local = 0.0
            noisy_dy_local = 0.0

        if kutno_kretanje:
            noisy_dyaw = (
                self.angular_scale * dyaw
                + self.generator.gauss(
                    0.0,
                    self.angular_noise_std
                ) * dt
            )
        else:
            noisy_dyaw = 0.0

        self.noisy_x += (
            math.cos(self.noisy_yaw) * noisy_dx_local
            - math.sin(self.noisy_yaw) * noisy_dy_local
        )

        self.noisy_y += (
            math.sin(self.noisy_yaw) * noisy_dx_local
            + math.cos(self.noisy_yaw) * noisy_dy_local
        )

        self.noisy_yaw = normalizacija_kuta(
            self.noisy_yaw + noisy_dyaw
        )

        noisy_vx = noisy_dx_local / dt
        noisy_vy = noisy_dy_local / dt
        noisy_wz = noisy_dyaw / dt

        self.zadnji_x = x
        self.zadnji_y = y
        self.zadnji_yaw = yaw
        self.zadnji_stamp = stamp

        self.objavi_odom(
            msg,
            noisy_vx,
            noisy_vy,
            noisy_wz
        )

    def objavi_odom(self, msg, noisy_vx, noisy_vy, noisy_wz):
        noisy_msg = copy.deepcopy(msg)

        noisy_msg.header.frame_id = (
            self.odom_frame
            if self.odom_frame
            else msg.header.frame_id
        )

        noisy_msg.child_frame_id = (
            self.base_link_frame
            if self.base_link_frame
            else msg.child_frame_id
        )

        noisy_msg.pose.pose.position.x = self.noisy_x
        noisy_msg.pose.pose.position.y = self.noisy_y
        noisy_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(
            0.0,
            0.0,
            self.noisy_yaw
        )

        noisy_msg.pose.pose.orientation.x = q[0]
        noisy_msg.pose.pose.orientation.y = q[1]
        noisy_msg.pose.pose.orientation.z = q[2]
        noisy_msg.pose.pose.orientation.w = q[3]

        noisy_msg.twist.twist.linear.x = noisy_vx
        noisy_msg.twist.twist.linear.y = noisy_vy
        noisy_msg.twist.twist.linear.z = 0.0

        noisy_msg.twist.twist.angular.x = 0.0
        noisy_msg.twist.twist.angular.y = 0.0
        noisy_msg.twist.twist.angular.z = noisy_wz

        noisy_msg.pose.covariance[0] = self.pose_covariance_xy
        noisy_msg.pose.covariance[7] = self.pose_covariance_xy
        noisy_msg.pose.covariance[35] = self.pose_covariance_yaw

        noisy_msg.twist.covariance[0] = (
            self.twist_covariance_linear
        )
        noisy_msg.twist.covariance[7] = (
            self.twist_covariance_lateral
        )
        noisy_msg.twist.covariance[35] = (
            self.twist_covariance_angular
        )

        self.odom_pub.publish(noisy_msg)

        if self.publish_tf:
            self.objavi_tf(noisy_msg)

    def objavi_tf(self, noisy_msg):
        parent_frame = noisy_msg.header.frame_id
        child_frame = noisy_msg.child_frame_id

        if not parent_frame:
            self.get_logger().error(
                'Nije zadan odom frame; TF nije objavljen.'
            )
            return

        if not child_frame:
            self.get_logger().error(
                'Nije zadan base_link frame; TF nije objavljen.'
            )
            return

        transform = TransformStamped()

        transform.header.stamp = noisy_msg.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        transform.transform.translation.x = self.noisy_x
        transform.transform.translation.y = self.noisy_y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = (
            noisy_msg.pose.pose.orientation.x
        )
        transform.transform.rotation.y = (
            noisy_msg.pose.pose.orientation.y
        )
        transform.transform.rotation.z = (
            noisy_msg.pose.pose.orientation.z
        )
        transform.transform.rotation.w = (
            noisy_msg.pose.pose.orientation.w
        )

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    node = NoisyOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()