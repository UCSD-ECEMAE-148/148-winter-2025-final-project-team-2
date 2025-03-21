import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np
import math
import time
import os


# Nodes in this program
NODE_NAME = 'subpub_lidar_actuator_node'

# Topics subcribed/published
LIDAR_TOPIC_NAME = '/scan'


class LidarActuatorSubpub(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        # adding multi-threading capability 
        self.lidar_thread = MutuallyExclusiveCallbackGroup()

        # Subsciber: Lidar
        self.laser_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, self.QUEUE_SIZE, callback_group=self.lidar_thread)
        self.laser_subscriber

        # Publisher: Actuator
        self.drive_pub = self.create_publisher(AckermannDriveStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()

        # setting up message structure for vesc-ackermann msg
        self.current_time = self.get_clock().now().to_msg()
        self.frame_id = 'base_link'

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
                ('live_laser_feed', 0),
                ('Ts', 0.05)
            ])

        # Update ROS parameters from config/launch file
        self.viewing_angle = self.get_parameter('viewing_angle').value
        self.front_degree_angle = self.get_parameter('front_degree_angle').value
        self.right_degree_angle = self.get_parameter('right_degree_angle').value
        self.left_degree_angle = self.get_parameter('left_degree_angle').value
        self.live_laser_feed = self.get_parameter('live_laser_feed').value
        self.Ts = self.get_parameter('Ts').value # controller sample time
        
        # Print ROS parameters
        self.get_logger().info(
            f'\n viewing_angle: {self.viewing_angle}'
            f'\n front_degree_angle: {self.front_degree_angle}'
            f'\n right_degree_angle: {self.right_degree_angle}'
            f'\n left_degree_angle: {self.left_degree_angle}'
            f'\n live_laser_feed: {self.live_laser_feed}'
            f'\n Ts: {self.Ts}'
        )

        # Call controller
        self.create_timer(self.Ts, self.controller)
    

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


    def controller(self):
        self.current_time = self.get_clock().now().to_msg()
        speed = 0
        delta = 0

        # publish drive control signal
        self.drive_cmd.header.stamp = self.current_time
        self.drive_cmd.header.frame_id = self.frame_id
        self.drive_cmd.drive.speed = speed
        self.drive_cmd.drive.steering_angle = delta
        self.drive_pub.publish(self.drive_cmd)


    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 


def main(args=None):
    rclpy.init(args=args)
    lidar_actuator_subpub = LidarActuatorSubpub()
    try:
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(lidar_actuator_subpub)
        try:
            executor.spin()
        finally:
            lidar_actuator_subpub.get_logger().info(f'Shutting down {NODE_NAME}...')
            lidar_actuator_subpub.drive_cmd.header.stamp = lidar_actuator_subpub.current_time
            lidar_actuator_subpub.drive_cmd.header.frame_id = lidar_actuator_subpub.frame_id
            lidar_actuator_subpub.drive_cmd.drive.speed = 0.0
            lidar_actuator_subpub.drive_cmd.drive.steering_angle = 0.0
            lidar_actuator_subpub.drive_pub.publish(lidar_actuator_subpub.drive_cmd)
            time.sleep(1)
            lidar_actuator_subpub.get_logger().info(f'{NODE_NAME} shut down successfully.')
            executor.shutdown()
            lidar_actuator_subpub.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()