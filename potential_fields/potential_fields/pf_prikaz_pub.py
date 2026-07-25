#!/usr/bin/env python3

import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener, TransformException
from scipy.ndimage import distance_transform_edt, gaussian_filter


class PotentialFieldPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('potential_field_pointcloud_publisher')

        self.declare_parameter('k_att', 1.0)
        self.declare_parameter('k_rep', 0.25)
        self.declare_parameter('rep_udaljenost', 0.8)
        self.declare_parameter('margine_regije', 4.0)
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('z_scale', 2.5)
        self.declare_parameter('z_offset', 0.0)
        self.declare_parameter('smooth_sigma', 0.8)
        self.declare_parameter('height_mode', 'sqrt')

        self.k_att = float(self.get_parameter('k_att').value)
        self.k_rep = float(self.get_parameter('k_rep').value)
        self.rep_udaljenost = float(self.get_parameter('rep_udaljenost').value)
        self.margine_regije = float(self.get_parameter('margine_regije').value)
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)
        self.z_scale = float(self.get_parameter('z_scale').value)
        self.z_offset = float(self.get_parameter('z_offset').value)
        self.smooth_sigma = float(self.get_parameter('smooth_sigma').value)
        self.height_mode = self.get_parameter('height_mode').value

        self.map_topic = '/map'
        self.cilj_topic = '/goal_pose'
        self.cloud_topic = '/robot_0/pf_oblak'
        self.global_frame = 'map'
        self.robot_base_frame = 'robot_0/base_link'

        self.zadnja_mapa = None
        self.cilj_x = None
        self.cilj_y = None
        self.ima_cilj = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(PoseStamped, self.cilj_topic, self.cilj_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 10)

    def map_callback(self, msg):
        self.zadnja_mapa = msg

    def cilj_callback(self, msg):
        self.cilj_x = msg.pose.position.x
        self.cilj_y = msg.pose.position.y
        self.ima_cilj = True
        self.objava_mape_polja()

    def dohvati_pozu_robota(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.robot_base_frame, Time())
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def napravi_robot_goal_region_masku(self, height, width, resolution, origin_x, origin_y, robot_x, robot_y):
        min_x = min(robot_x, self.cilj_x) - self.margine_regije
        max_x = max(robot_x, self.cilj_x) + self.margine_regije
        min_y = min(robot_y, self.cilj_y) - self.margine_regije
        max_y = max(robot_y, self.cilj_y) + self.margine_regije
        xs = origin_x + (np.arange(width) + 0.5) * resolution
        ys = origin_y + (np.arange(height) + 0.5) * resolution
        grid_x, grid_y = np.meshgrid(xs, ys)
        return (grid_x >= min_x) & (grid_x <= max_x) & (grid_y >= min_y) & (grid_y <= max_y)

    def pack_rgb(self, r, g, b):
        rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
        return struct.unpack('f', struct.pack('I', rgb_uint32))[0]

    def boja_iz_normaliziranog_potencijala(self, t):
        t = float(np.clip(t, 0.0, 1.0))

        if t < 0.25:
            a = t / 0.25
            return 0, int(255 * a), 255

        if t < 0.50:
            a = (t - 0.25) / 0.25
            return 0, 255, int(255 * (1.0 - a))

        if t < 0.75:
            a = (t - 0.50) / 0.25
            return int(255 * a), 255, 0

        a = (t - 0.75) / 0.25
        return 255, int(255 * (1.0 - a)), 0

    def zagladi_potencijal(self, potential, valid):
        if self.smooth_sigma <= 0.0 or not np.any(valid):
            return potential

        filled = np.array(potential, copy=True)
        filled[~valid] = 0.0
        weights = valid.astype(np.float32)
        blurred_values = gaussian_filter(filled, sigma=self.smooth_sigma)
        blurred_weights = gaussian_filter(weights, sigma=self.smooth_sigma)
        smoothed = np.array(potential, copy=True)
        mask = blurred_weights > 1e-6
        smoothed[mask] = blurred_values[mask] / blurred_weights[mask]
        smoothed[~valid] = np.nan
        return smoothed

    def transformiraj_visinu(self, norm):
        if self.height_mode == 'sqrt':
            return np.sqrt(norm)
        return norm

    def objava_mape_polja(self):
        if self.zadnja_mapa is None or not self.ima_cilj or self.cilj_x is None or self.cilj_y is None:
            return

        mapa = self.zadnja_mapa
        width = mapa.info.width
        height = mapa.info.height
        resolution = mapa.info.resolution

        if width == 0 or height == 0 or resolution <= 0.0:
            return

        origin_x = mapa.info.origin.position.x
        origin_y = mapa.info.origin.position.y
        robot_pose = self.dohvati_pozu_robota()

        if robot_pose is None:
            return

        robot_x, robot_y = robot_pose
        data = np.array(mapa.data, dtype=np.int16).reshape((height, width))

        obstacle_mask = data > self.obstacle_threshold
        free_mask = (data <= self.obstacle_threshold) & (data != -1)
        region_mask = self.napravi_robot_goal_region_masku(height, width, resolution, origin_x, origin_y, robot_x, robot_y)
        free_mask = free_mask & region_mask

        if not np.any(free_mask):
            return

        xs = origin_x + (np.arange(width) + 0.5) * resolution
        ys = origin_y + (np.arange(height) + 0.5) * resolution
        grid_x, grid_y = np.meshgrid(xs, ys)

        dist_goal = np.hypot(self.cilj_x - grid_x, self.cilj_y - grid_y)
        u_att = 0.5 * self.k_att * dist_goal * dist_goal

        if self.k_rep > 0.0 and np.any(obstacle_mask):
            dist_to_obstacle = distance_transform_edt(~obstacle_mask) * resolution
            u_rep = np.zeros_like(u_att, dtype=np.float32)
            rep_mask = free_mask & (dist_to_obstacle > 0.0) & (dist_to_obstacle < self.rep_udaljenost)
            u_rep[rep_mask] = 0.5 * self.k_rep * ((1.0 / dist_to_obstacle[rep_mask]) - (1.0 / self.rep_udaljenost)) ** 2
        else:
            u_rep = np.zeros_like(u_att, dtype=np.float32)

        potential = u_att + u_rep
        potential[~free_mask] = np.nan
        valid = ~np.isnan(potential)

        if not np.any(valid):
            return

        potential = self.zagladi_potencijal(potential, valid)
        valid = ~np.isnan(potential)

        if not np.any(valid):
            return

        min_p = float(np.nanmin(potential[valid]))
        max_p = float(np.nanmax(potential[valid]))
        norm = np.zeros_like(potential, dtype=np.float32)

        if max_p > min_p:
            norm[valid] = (potential[valid] - min_p) / (max_p - min_p)

        norm[valid] = self.transformiraj_visinu(norm[valid])

        points = []

        for gy in range(height):
            for gx in range(width):
                if not valid[gy, gx]:
                    continue

                x = float(grid_x[gy, gx])
                y = float(grid_y[gy, gx])
                z = float(self.z_offset + self.z_scale * norm[gy, gx])
                r, g, b = self.boja_iz_normaliziranog_potencijala(norm[gy, gx])
                rgb = self.pack_rgb(r, g, b)
                points.append([x, y, z, rgb])

        if len(points) == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = mapa.header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        self.cloud_pub.publish(point_cloud2.create_cloud(header, fields, points))
        self.get_logger().info("Objavljena je mapa potencijalnih polja.")


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPointCloudPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()