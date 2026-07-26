#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SimulatedImuNode(Node):
    def __init__(self):
        super().__init__('imu_sim_node')

        self.declare_parameter('input_ground_truth_topic', 'ground_truth')
        self.declare_parameter('output_imu_topic', 'imu/data')
        self.declare_parameter('imu_frame', '')

        self.declare_parameter('gyro_scale', 1.005)
        self.declare_parameter('gyro_bias', 0.0)
        self.declare_parameter('gyro_noise_std', 0.0005)
        self.declare_parameter('gyro_covariance', 0.000025)
        self.declare_parameter('random_seed', 2001)

        self.input_ground_truth_topic = str(self.get_parameter('input_ground_truth_topic').value)
        self.output_imu_topic = str(self.get_parameter('output_imu_topic').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)

        self.gyro_scale = float(self.get_parameter('gyro_scale').value)
        self.gyro_bias = float(self.get_parameter('gyro_bias').value)
        self.gyro_noise_std = float(self.get_parameter('gyro_noise_std').value)
        self.gyro_covariance = float(self.get_parameter('gyro_covariance').value)

        self.random_seed = int(self.get_parameter('random_seed').value)

        if self.random_seed >= 0:
            self.generator = random.Random(self.random_seed)
        else:
            self.generator = random.Random()

        self.imu_pub = self.create_publisher(Imu, self.output_imu_topic, 10)

        self.ground_truth_sub = self.create_subscription(Odometry, self.input_ground_truth_topic, self.ground_truth_callback, 10)

    def ground_truth_callback(self, msg):
        stvarni_wz = msg.twist.twist.angular.z
        gyro_noise = self.generator.gauss(0.0, self.gyro_noise_std)

        izmjereni_wz = self.gyro_scale * stvarni_wz + self.gyro_bias + gyro_noise

        imu_msg = Imu()

        imu_msg.header.stamp = msg.header.stamp

        if self.imu_frame:
            imu_msg.header.frame_id = self.imu_frame
        elif msg.child_frame_id:
            imu_msg.header.frame_id = msg.child_frame_id
        else:
            imu_msg.header.frame_id = 'base_link'

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        imu_msg.orientation_covariance[0] = -1.0

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = izmjereni_wz

        imu_msg.angular_velocity_covariance[0] = 1000000.0
        imu_msg.angular_velocity_covariance[4] = 1000000.0
        imu_msg.angular_velocity_covariance[8] = self.gyro_covariance

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    node = SimulatedImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()