import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np
import math
import time
import os


# Nodes in this program
NODE_NAME = 'sub_lidar_node'

# Topics subcribed/published
LIDAR_TOPIC_NAME = '/scan'


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        # Subsciber: Lidar
        self.laser_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, self.QUEUE_SIZE)
        self.laser_subscriber

        # Lidar info
        self.lidar_properties_set = False
        self.default_viewing_angle = 360
        self.default_front_degree_angle = 0
        self.default_right_degree_angle = 90
        self.default_left_degree_angle = 270
        self.range_max = None
        self.range_min = None
        self.num_scans = None
        self.angle_increment = None
        self.angle_min_radians = None
        self.angle_max_radians = None
        self.scan_ranges = None

        # ROS parameters (setttng default values)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('viewing_angle', self.default_viewing_angle),
                ('front_degree_angle', self.default_front_degree_angle),
                ('right_degree_angle', self.default_right_degree_angle),
                ('left_degree_angle', self.default_left_degree_angle),
                ('live_laser_feed', 0)
            ])

        # Update ROS parameters from config/launch file
        self.viewing_angle = self.get_parameter('viewing_angle').value
        self.front_degree_angle = self.get_parameter('front_degree_angle').value
        self.right_degree_angle = self.get_parameter('right_degree_angle').value
        self.left_degree_angle = self.get_parameter('left_degree_angle').value
        self.live_laser_feed = self.get_parameter('live_laser_feed').value
        
        # Print ROS parameters
        self.get_logger().info(
            f'\n viewing_angle: {self.viewing_angle}'
            f'\n front_degree_angle: {self.front_degree_angle}'
            f'\n right_degree_angle: {self.right_degree_angle}'
            f'\n left_degree_angle: {self.left_degree_angle}'
            f'\n live_laser_feed: {self.live_laser_feed}'
        )
    

    def lidar_callback(self, data):
        if not self.lidar_properties_set:
            self.range_max = data.range_max
            self.range_min = data.range_min
            self.num_scans = len(np.array(data.ranges))
            self.angle_increment = data.angle_increment
            self.angle_min_radians = data.angle_min
            self.angle_max_radians = data.angle_max
            self.lidar_properties_set = True
        
        scan_ranges = np.array(data.ranges)
        self.scan_ranges = np.nan_to_num(scan_ranges, neginf=self.range_max, posinf=self.range_max, nan=self.range_max)

        # printing distances from lidar measurements
        if self.live_laser_feed:
            self.get_logger().info(
                f'\n scan_ranges: {self.scan_ranges}'
                )
            

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    try:
        rclpy.spin(lidar_subscriber)
        lidar_subscriber.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        lidar_subscriber.get_logger().info(f'Shutting down {NODE_NAME}...')
        time.sleep(1)
        lidar_subscriber.destroy_node()
        rclpy.shutdown()
        lidar_subscriber.get_logger().info(f'{NODE_NAME} shut down successfully.')


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    try:
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(lidar_subscriber)
        try:
            executor.spin()
        finally:
            lidar_subscriber.get_logger().info(f'Shutting down {NODE_NAME}...')
            time.sleep(1)
            lidar_subscriber.get_logger().info(f'{NODE_NAME} shut down successfully.')
            executor.shutdown()
            lidar_subscriber.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
