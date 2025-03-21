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

#Topics & Subs, Pubs
LIDAR_TOPIC_NAME = '/scan'
ODOM_TOPIC_NAME = '/odom'
ERROR_TOPIC_NAME = '/error'
NODE_NAME = 'tube_follower_node'


class TubeFollower(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.debug = False
        self.QUEUE_SIZE = 10
        # self.imu_thread = MutuallyExclusiveCallbackGroup()

        # Get Odometry measurements
        self.odom_subscriber = self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odom_measurement, self.QUEUE_SIZE)
        self.odom_subscriber
        
        # Get IMU measurement
        # self.imu_subscriber = self.create_subscription(Imu, IMU_TOPIC_NAME, self.imu_measurement, rclpy.qos.qos_profile_sensor_data, callback_group=self.imu_thread)
        # self.imu_subscriber

        # Get Laser Data
        self.laser_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, 10)
        self.laser_subscriber
        
        # Setting up publisher
        self.error_publisher = self.create_publisher(Float32MultiArray, ERROR_TOPIC_NAME, 10)
        self.error_publisher
        self.error_msg = Float32MultiArray() # [error_distance_to_wall, heading_error_with_wall]
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        # self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim[0].label = "lateral_error"
        self.error_msg.layout.dim[1].label = "longitdudinal_error"
        self.error_msg.layout.dim[2].label = "heading_error"
        # self.error_msg.layout.dim[3].label = "future_curvature"
        self.error_msg.layout.dim[0].size = 1
        self.error_msg.layout.dim[1].size = 1
        self.error_msg.layout.dim[2].size = 1
        # self.error_msg.layout.dim[3].size = 1

        # Lidar info
        self.default_desired_lateral_distance = 0
        self.default_desired_longitudinal_distance = 1
        self.default_viewing_angle = 360
        self.default_degree_window_size = 0
        self.default_front_degree_angle = 0
        self.default_right_degree_angle = 90
        self.default_left_degree_angle = 270
        self.default_gamma_offset_max = 0
        self.default_path_difference_threshold = 0
        self.default_path_heading_threshold = 5
        self.default_tc = 0.1
        self.max_range = None
        self.phi = 0

        self.num_scans = 0
        self.scans_per_degree = 0
        self.lateral_degree_window = 0
        self.lateral_start_angle = 0
        self.lateral_end_angle = 0
        self.longitudinal_degree_window = 0
        self.longitudinal_start_angle = 0
        self.longitudinal_end_angle = 0
        self.vx = 0.1

        # wall following parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_lateral_distance', self.default_desired_lateral_distance),
                ('desired_longitudinal_distance', self.default_desired_longitudinal_distance),
                ('viewing_angle', self.default_viewing_angle),
                ('degree_window_size', self.default_degree_window_size),
                ('front_degree_angle', self.default_front_degree_angle),
                ('right_degree_angle', self.default_right_degree_angle),
                ('left_degree_angle', self.default_left_degree_angle),
                ('gamma_offset_max', self.default_gamma_offset_max),
                ('path_difference_threshold', self.default_path_difference_threshold),
                ('path_heading_threshold', self.default_path_heading_threshold),
                ('tc', self.default_tc),
            ])

        self.reff_lat_wall_dist = self.get_parameter('desired_lateral_distance').value
        self.reff_lon_wall_dist = self.get_parameter('desired_longitudinal_distance').value
        self.viewing_angle = self.get_parameter('viewing_angle').value
        self.degree_window_size = self.get_parameter('degree_window_size').value
        self.front_degree_angle = self.get_parameter('front_degree_angle').value
        self.right_degree_angle = self.get_parameter('right_degree_angle').value
        self.left_degree_angle = self.get_parameter('left_degree_angle').value
        self.gamma_offset_max = self.get_parameter('gamma_offset_max').value
        self.path_difference_threshold = self.get_parameter('path_difference_threshold').value
        self.path_heading_threshold = self.get_parameter('path_heading_threshold').value
        self.tc = self.get_parameter('tc').value
        
    def odom_measurement(self, odom_data):
        # car linear velocity
        self.vx = odom_data.twist.twist.linear.x
        
        # if self.debug:
        #     self.get_logger().info(f"Updating odom: {self.vx}")

    def get_wall_distance(self, degree_window, angle_offset=0):
        # remove nan/inf
        degree_window = self.remove_nan_inf(degree_window)
            
        # Distance to current right wall
        average_distance = np.mean(degree_window) * math.cos(math.radians(angle_offset))
        
        # if self.debug:
        #     self.get_logger().info(f"Updating wall distance: {average_distance}")
        return average_distance
            
    def get_wall_heading(self, degree_window, angle_window, angle_offset):
        # remove nan/inf
        degree_window = self.remove_nan_inf(degree_window)
        
        # path angle distances
        x1 = float(degree_window[0]) * math.cos(math.radians(angle_window + angle_offset))
        x2 = float(degree_window[-1]) * math.cos(math.radians(angle_window + angle_offset))
        y1 = float(degree_window[0]) * math.sin(math.radians(angle_window + angle_offset))
        y2 = float(degree_window[-1]) * math.sin(math.radians(angle_window + angle_offset))

        delta_x = x2 - x1 
        delta_y = y1 + y2
        path_heading = abs(float(np.arctan2(delta_x, delta_y)))
        sign_direction = float(np.sign(delta_x))
        if sign_direction == 0.0:
            sign_direction = 1.0
        path_arc_length = np.linalg.norm([delta_x, delta_y])
        
        # if self.debug:
        #     self.get_logger().info(f"Updating wall heading: {angle_offset}, {path_heading}, {delta_x}, {path_arc_length}, {sign_direction}, {path_arc_length}")
        return path_heading, sign_direction, path_arc_length
    
    def remove_nan_inf(self,degree_window):
        # remove nan/inf
        degree_window = degree_window[~np.isnan(degree_window)]
        degree_window = degree_window[~np.isinf(degree_window)]
        return degree_window

    def lidar_callback(self, data):
        if self.max_range is None:
            self.max_range = data.range_max
        scan_ranges = np.array(data.ranges)
        # scan_ranges = scan_ranges[~np.isnan(scan_ranges)]
        # degree_window = degree_window[~np.isinf(degree_window)]
        self.num_scans = len(scan_ranges)
        self.scans_per_degree = float(self.num_scans/self.viewing_angle)
        # self.get_logger().info(f'num_scans,scans_per_degree,viewing_angle:{self.num_scans},{self.scans_per_degree},{self.viewing_angle}')
        theta_w = self.degree_window_size / 2

        try:
            ############################# Longitudinal ############################
            
            # Distance to current front wall
            longitudinal_start_angle = self.front_degree_angle - theta_w
            longitudinal_end_angle = self.front_degree_angle + theta_w
            long_start = np.array(scan_ranges[int(longitudinal_start_angle*self.scans_per_degree):])
            long_end = np.array(scan_ranges[:int(longitudinal_end_angle*self.scans_per_degree)])
            longitudinal_degree_window = np.concatenate([long_start, long_end])
            front_average_distance = self.get_wall_distance(longitudinal_degree_window, angle_offset=0)
            
            # Look ahead angle based on measured frontal distance
            gamma_offset = (self.gamma_offset_max / self.max_range) * front_average_distance
            
            gamma_offset = 15

            ############################### Current Lateral ############################### 
            
            # Distance to current right wall
            lateral_right_start_angle = self.right_degree_angle - theta_w + gamma_offset
            lateral_right_end_angle = self.right_degree_angle + theta_w + gamma_offset
            lateral_right_degree_window = np.array(scan_ranges[int(lateral_right_start_angle*self.scans_per_degree):int(lateral_right_end_angle*self.scans_per_degree)])
            right_average_side_distance = self.get_wall_distance(lateral_right_degree_window, angle_offset=gamma_offset)

            # Heading of current right wall
            right_path_heading, right_sign_direction, _ = self.get_wall_heading(lateral_right_degree_window, theta_w, angle_offset=gamma_offset)

            # Distance to delayed - left wall
            lateral_left_delayed_start_angle = self.left_degree_angle - theta_w + gamma_offset
            lateral_left_delayed_end_angle = self.left_degree_angle + theta_w + gamma_offset
            lateral_left_delayed_degree_window = np.array(scan_ranges[int(lateral_left_delayed_start_angle*self.scans_per_degree):int(lateral_left_delayed_end_angle*self.scans_per_degree)])
            left_delayed_average_side_distance = self.get_wall_distance(lateral_left_delayed_degree_window, angle_offset=gamma_offset)

            # Distance to current - left wall (Making correction)
            phi = np.arctan((self.vx * self.tc) / left_delayed_average_side_distance)
            lateral_left_start_angle = self.left_degree_angle - theta_w - phi + gamma_offset
            lateral_left_end_angle = self.left_degree_angle + theta_w - phi + gamma_offset
            lateral_left_degree_window = np.array(scan_ranges[int(lateral_left_start_angle*self.scans_per_degree):int(lateral_left_end_angle*self.scans_per_degree)])
            left_average_side_distance = self.get_wall_distance(lateral_left_degree_window, angle_offset=(phi+gamma_offset))

            # Heading of current left wall
            left_path_heading, _, _  = self.get_wall_heading(lateral_left_degree_window, theta_w, angle_offset=(phi+gamma_offset))

            ############################### errors for control ############################### 
                        
            # Longitduinal
            longitdudinal_error = -1 * min(0, (front_average_distance - self.reff_lon_wall_dist))
            
            # Current Lateral
            path_difference = abs(right_path_heading - left_path_heading)
            if path_difference > math.radians(self.path_difference_threshold):
                # heading_error = -1 * np.sign(right_sign_direction) * np.max([right_path_heading, left_path_heading])
                heading_error = -1 * np.sign(right_sign_direction) * np.min([right_path_heading, left_path_heading])
                if self.debug:
                    self.get_logger().info(f"Here 1.13 {heading_error}")
            else:
                heading_error = -1 * np.sign(right_sign_direction) * np.mean([right_path_heading, left_path_heading])
                if self.debug:
                    self.get_logger().info(f"Here 1.12 {heading_error}")
            lateral_difference = (left_average_side_distance - right_average_side_distance) / 2
            lateral_error = (self.reff_lat_wall_dist - lateral_difference) * math.cos(heading_error)
            
            self.error_msg.data = [float(lateral_error), float(longitdudinal_error), float(heading_error)]
            self.error_publisher.publish(self.error_msg)
            if self.debug:
                self.get_logger().info(f"Here 17 {self.error_msg.data}")
            
        except IndexError or ValueError:
            print(f"got nan or inf still or index?")
            
            
            # if self.debug:
            #     self.get_logger().info(f'\n'
            #                         f'\n lateral_right_degree_window:{lateral_right_degree_window}'
            #                         f'\n lateral_left_degree_window:{lateral_left_degree_window}'
            #                         f'\n longitudinal_degree_window:{longitudinal_degree_window}'
            #                         f'\n right_average_side_distance:{right_average_side_distance}'
            #                         f'\n left_average_side_distance:{left_average_side_distance}'
            #                         f'\n front_average_distance:{front_average_distance}'
            #                         f'\n lateral_difference:{lateral_difference}'
            #                         f'\n right start angle:{lateral_right_start_angle}'
            #                         f'\n right end angle:{lateral_right_end_angle}'
            #                         f'\n left start angle:{lateral_left_start_angle}'
            #                         f'\n left end angle:{lateral_left_end_angle}'
            #                         f'\n left start distance:{lateral_left_degree_window[0]}'
            #                         f'\n left end distance:{lateral_left_degree_window[-1]}'
            #                         f'\n right start distance:{lateral_right_degree_window[0]}'
            #                         f'\n right end distance:{lateral_right_degree_window[-1]}'
            #                         f'\n '
            #                         f'\n '
            #                         f'\n delta_x_right:{delta_x_right}'
            #                         f'\n delta_y_right:{delta_y_right}'
            #                         f'\n right_path_heading:{right_path_heading}'
            #                         f'\n '
            #                         f'\n '
            #                         f'\n delta_x_left:{delta_x_left}'
            #                         f'\n delta_y_left:{delta_y_left}'
            #                         f'\n left_path_heading:{left_path_heading}'
            #                         f'\n '
            #                         f'\n '
            #                         f'\n heading_error:{heading_error}'
            #                             )
            
            # if self.debug:
            #     self.get_logger().info(f"Here 12")
            # # Future Lateral
            # future_path_difference = abs(future_right_path_heading - future_left_path_heading)
            # future_lateral_difference = (future_left_average_side_distance - future_right_average_side_distance) / 2
            # future_lateral_error = self.reff_lat_wall_dist - future_lateral_difference
            
            # if self.debug:
            #     self.get_logger().info(f"Here 13")
            # arc_length_array = np.array([future_right_path_arc_length, future_left_path_arc_length])
            # path_heading_array = np.array([future_right_path_heading, future_left_path_heading])
            # if self.debug:
            #     self.get_logger().info(f"Here 14")
            
            # if future_path_difference > math.radians(self.path_difference_threshold):
            #     future_heading_error = -1 * np.sign(future_right_sign_direction) * np.min(path_heading_array)
            #     future_path_arc_length = arc_length_array[np.argmax(path_heading_array)]
            # else:
            #     future_heading_error = -1 * np.sign(future_right_sign_direction) * np.mean([right_path_heading, future_left_path_heading])
            #     future_path_arc_length = np.mean(arc_length_array)
                
            # if future_heading_error < math.radians(self.path_heading_threshold):
            #     future_heading_error = 0.0
            # if self.debug:
            #     self.get_logger().info(f"Here 15")

            # future_curvature = abs(future_heading_error / future_path_arc_length)
            # if self.debug:
            #     self.get_logger().info(f"Here 16:"
            #         f"\n future_heading_error: {future_heading_error}"
            #         f"\n future_path_arc_length: {future_path_arc_length}"
            #         f"\n future_right_path_arc_length: {future_right_path_arc_length}"
            #         f"\n future_left_path_arc_length: {future_left_path_arc_length}"
            #         f"\n future_left_path_heading: {future_left_path_heading}"
            #         f"\n future_right_path_heading: {future_right_path_heading}"
            #         f"\n future_curvature: {future_curvature}")
            
            # Publish
            # self.error_msg.data = [float(lateral_error), float(longitdudinal_error), float(heading_error), float(future_curvature)]
        #     self.error_msg.data = [float(lateral_error), float(longitdudinal_error), float(heading_error)]
        #     self.error_publisher.publish(self.error_msg)
        #     if self.debug:
        #         self.get_logger().info(f"Here 17 {self.error_msg.data}")
            
        # except IndexError or ValueError:
        #     print(f"got nan or inf still or index?")


def main(args=None):
    rclpy.init(args=args)
    tube_follower_publisher = TubeFollower()
    try:
        rclpy.spin(tube_follower_publisher)
        tube_follower_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        tube_follower_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        time.sleep(1)
        tube_follower_publisher.destroy_node()
        rclpy.shutdown()
        tube_follower_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
