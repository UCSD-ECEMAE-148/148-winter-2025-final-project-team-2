# def get_wall_distance(self, degree_window, angle_offset=0):
#     # remove nan/inf
#     degree_window = self.remove_nan_inf(degree_window)
        
#     # Distance to current right wall
#     average_distance = np.mean(degree_window) * math.cos(math.radians(angle_offset))
#     return average_distance
        

# def get_wall_heading(self, degree_window, angle_window, angle_offset=0):
#     # remove nan/inf
#     degree_window = self.remove_nan_inf(degree_window)
    
#     # path angle distances
#     x1 = float(degree_window[0]) * math.cos(math.radians(angle_window + angle_offset))
#     x2 = float(degree_window[-1]) * math.cos(math.radians(angle_window + angle_offset))
#     y1 = float(degree_window[0]) * math.sin(math.radians(angle_window + angle_offset))
#     y2 = float(degree_window[-1]) * math.sin(math.radians(angle_window + angle_offset))
#     delta_x = x2 - x1 
#     delta_y = y1 + y2
#     path_heading = abs(float(np.arctan2(delta_x, delta_y)))
#     return path_heading

# def remove_nan_inf(self,degree_window):
#     # remove nan/inf
#     degree_window = degree_window[~np.isnan(degree_window)]
#     degree_window = degree_window[~np.isinf(degree_window)]
#     return degree_window
    

# def get_wall_distance_and_heading(self):
#     pass


# def lidar_measure(self)
#     # Distance to current right wall
#     lateral_right_start_angle = self.right_degree_angle - theta_w
#     lateral_right_end_angle = self.right_degree_angle + theta_w
#     lateral_right_degree_window = np.array(scan_ranges[int(lateral_right_start_angle*self.scans_per_degree):int(lateral_right_end_angle*self.scans_per_degree)])
        
#     # Heading of current right wall
#     right_average_side_distance = self.get_wall_distance(lateral_right_degree_window, angle_offset=0)
#     right_path_heading = self.get_wall_heading(lateral_right_degree_window, theta_w, angle_offset=0)
    
#     # Distance to delayed - left wall
#     lateral_left_delayed_start_angle = self.left_degree_angle - theta_w
#     lateral_left_delayed_end_angle = self.left_degree_angle + theta_w
#     lateral_left_delayed_degree_window = np.array(scan_ranges[int(lateral_left_delayed_start_angle*self.scans_per_degree):int(lateral_left_delayed_end_angle*self.scans_per_degree)])
#     left_delayed_average_side_distance = self.get_wall_distance(lateral_left_delayed_degree_window, angle_offset=0)
    
#     # Distance to current left wall - Making correction
#     lateral_left_start_angle = self.left_degree_angle - theta_w - phi
#     lateral_left_end_angle = self.left_degree_angle + theta_w - phi
#     lateral_left_degree_window = np.array(scan_ranges[int(lateral_left_start_angle*self.scans_per_degree):int(lateral_left_end_angle*self.scans_per_degree)])
#     # Heading of current left wall
#     phi = np.arctan((self.vx * self.tc) / left_delayed_average_side_distance)
#     left_average_side_distance = self.get_wall_distance(lateral_left_degree_window, angle_offset=0)
#     left_path_heading = self.get_wall_heading(lateral_left_degree_window, theta_w, angle_offset=phi)
    
    
    
    
        
        
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
# import numpy as np
# import math
# import time

# #Topics & Subs, Pubs
# LIDAR_TOPIC_NAME = '/scan'
# ERROR_TOPIC_NAME = '/error'
# NODE_NAME = 'tube_follower_node'

# class TubeFollower(Node):
#     def __init__(self):
#         super().__init__(NODE_NAME)
#         self.camera_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, 10)
#         self.camera_subscriber
#         self.error_publisher = self.create_publisher(Float32MultiArray, ERROR_TOPIC_NAME, 10)
#         self.error_publisher
#         self.error_msg = Float32MultiArray() # [error_distance_to_wall, heading_error_with_wall]
#         self.error_msg.layout.dim.append(MultiArrayDimension())
#         self.error_msg.layout.dim.append(MultiArrayDimension())
#         self.error_msg.layout.dim.append(MultiArrayDimension())
#         self.error_msg.layout.dim[0].label = "lateral_error"
#         self.error_msg.layout.dim[1].label = "longitdudinal_error"
#         self.error_msg.layout.dim[2].label = "heading_error"
#         self.error_msg.layout.dim[3].label = "predicted_heading_error"
#         self.error_msg.layout.dim[0].size = 1
#         self.error_msg.layout.dim[1].size = 1
#         self.error_msg.layout.dim[2].size = 1
#         self.error_msg.layout.dim[3].size = 1

#         # Lidar info
#         self.default_desired_lateral_distance = 1
#         self.default_desired_longitudinal_distance = 1
#         self.default_viewing_angle = 360
#         self.default_degree_window_size = 0
#         self.default_front_degree_angle = 0
#         self.default_right_degree_angle = 90
#         self.default_left_degree_angle = 270
#         self.default_gamma_offset_max = 0
#         self.default_path_difference_threshold = 0
#         self.max_range = None
#         self.phi = 0

#         self.num_scans = 0
#         self.scans_per_degree = 0
#         self.lateral_degree_window = 0
#         self.lateral_start_angle = 0
#         self.lateral_end_angle = 0
#         self.longitudinal_degree_window = 0
#         self.longitudinal_start_angle = 0
#         self.longitudinal_end_angle = 0

#         # wall following parameters
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('desired_lateral_distance', self.default_desired_lateral_distance),
#                 ('desired_longitudinal_distance', self.default_desired_longitudinal_distance),
#                 ('viewing_angle', self.default_viewing_angle),
#                 ('degree_window_size', self.default_degree_window_size),
#                 ('front_degree_angle', self.default_front_degree_angle),
#                 ('right_degree_angle', self.default_right_degree_angle),
#                 ('left_degree_angle', self.default_left_degree_angle),
#                 ('gamma_offset_max', self.default_gamma_offset_max),
#                 ('path_difference_threshold', self.default_path_difference_threshold)
#             ])

#         self.reff_lat_wall_dist = self.get_parameter('desired_lateral_distance').value
#         self.reff_lon_wall_dist = self.get_parameter('desired_longitudinal_distance').value
#         self.viewing_angle = self.get_parameter('viewing_angle').value
#         self.degree_window_size = self.get_parameter('degree_window_size').value
#         self.front_degree_angle = self.get_parameter('front_degree_angle').value
#         self.right_degree_angle = self.get_parameter('right_degree_angle').value
#         self.left_degree_angle = self.get_parameter('left_degree_angle').value
#         self.gamma_offset_max = self.get_parameter('gamma_offset_max').value
#         self.path_difference_threshold = self.get_parameter('path_difference_threshold').value
        
#     def get_wall_distance(self, degree_window, angle_offset=0):
#         # remove nan/inf
#         degree_window = self.remove_nan_inf(degree_window)
            
#         # Distance to current right wall
#         average_distance = np.mean(degree_window) * math.cos(math.radians(angle_offset))
#         return average_distance
            
#     def get_wall_heading(self, degree_window, angle_window, angle_offset=0):
#         # remove nan/inf
#         degree_window = self.remove_nan_inf(degree_window)
        
#         # path angle distances
#         x1 = float(degree_window[0]) * math.cos(math.radians(angle_window + angle_offset))
#         x2 = float(degree_window[-1]) * math.cos(math.radians(angle_window + angle_offset))
#         y1 = float(degree_window[0]) * math.sin(math.radians(angle_window + angle_offset))
#         y2 = float(degree_window[-1]) * math.sin(math.radians(angle_window + angle_offset))

#         delta_x = x2 - x1 
#         delta_y = y1 + y2
#         path_heading = abs(float(np.arctan2(delta_x, delta_y)))
#         return path_heading
    
#     def remove_nan_inf(self,degree_window):
#         # remove nan/inf
#         degree_window = degree_window[~np.isnan(degree_window)]
#         degree_window = degree_window[~np.isinf(degree_window)]
#         return degree_window

#     def lidar_callback(self, data):
#         if self.max_range is None:
#             self.max_range = data.range_max
#         scan_ranges = np.array(data.ranges)
#         scan_ranges = scan_ranges[~np.isnan(scan_ranges)]
#         self.num_scans = len(scan_ranges)
#         self.scans_per_degree = float(self.num_scans/self.viewing_angle)
#         # self.get_logger().info(f'num_scans,scans_per_degree,viewing_angle:{self.num_scans},{self.scans_per_degree},{self.viewing_angle}')
#         theta_w = self.degree_window_size / 2
        
#         try:
#             ############################# Longitudinal ############################
#             # Viewing windows
#             longitudinal_start_angle = self.front_degree_angle - theta_w
#             longitudinal_end_angle = self.front_degree_angle + theta_w
#             long_start = np.array(scan_ranges[int(longitudinal_start_angle*self.scans_per_degree):])
#             long_end = np.array(scan_ranges[:int(longitudinal_end_angle*self.scans_per_degree)])
#             longitudinal_degree_window = np.concatenate([long_start, long_end])
            
#             # remove nan/inf
#             longitudinal_degree_window = longitudinal_degree_window[~np.isnan(longitudinal_degree_window)]
#             longitudinal_degree_window = longitudinal_degree_window[~np.isinf(longitudinal_degree_window)]
            
#             # Distance to front wall
#             front_average_distance = np.mean(longitudinal_degree_window)
#             gamma_offset = (self.gamma_offset_max / self.max_range) * front_average_distance
        
#             ############################### Current Lateral ############################### 
            
#             ######### Viewing windows : right
#             lateral_right_start_angle = gamma_offset + self.right_degree_angle - theta_w
#             lateral_right_end_angle = gamma_offset + self.right_degree_angle + theta_w
#             lateral_right_degree_window = np.array(scan_ranges[int(lateral_right_start_angle*self.scans_per_degree):int(lateral_right_end_angle*self.scans_per_degree)])
            
#             # remove nan/inf
#             lateral_right_degree_window = lateral_right_degree_window[~np.isnan(lateral_right_degree_window)]
#             lateral_right_degree_window = lateral_right_degree_window[~np.isinf(lateral_right_degree_window)]
            
#             # right path angle distances
#             x1 = float(lateral_right_degree_window[0]) * math.cos(math.radians(theta_w + gamma_offset))
#             x2 = float(lateral_right_degree_window[-1]) * math.cos(math.radians(theta_w + gamma_offset))
#             y1 = float(lateral_right_degree_window[0]) * math.sin(math.radians(theta_w + gamma_offset))
#             y2 = float(lateral_right_degree_window[-1]) * math.sin(math.radians(theta_w + gamma_offset))

#             delta_x_right = x2 - x1 
#             delta_y_right = y1 + y2
            
#             # Distance to current right wall
#             right_average_side_distance = np.mean(lateral_right_degree_window) * math.cos(math.radians(gamma_offset))
            
#             ######### Viewing windows : left
#             # delayed left wall measurement
#             lateral_left_start_angle = gamma_offset + self.left_degree_angle - theta_w
#             lateral_left_end_angle = gamma_offset + self.left_degree_angle + theta_w
#             lateral_left_degree_window = np.array(scan_ranges[int(lateral_left_start_angle*self.scans_per_degree):int(lateral_left_end_angle*self.scans_per_degree)])

#             # remove nan/inf
#             lateral_left_degree_window = lateral_left_degree_window[~np.isnan(lateral_left_degree_window)]
#             lateral_left_degree_window = lateral_left_degree_window[~np.isinf(lateral_left_degree_window)]

#             # Distance to delayed - left wall
#             left_delayed_average_side_distance = np.mean(lateral_left_degree_window) * math.cos(math.radians(gamma_offset))
            
#             ######### left - Making correction
#             phi = np.arctan((self.vx * self.tc) / left_delayed_average_side_distance)

#             # Viewing windows : left - corrected
#             lateral_left_start_angle = gamma_offset + self.left_degree_angle - theta_w - phi
#             lateral_left_end_angle = gamma_offset + self.left_degree_angle + theta_w - phi
#             lateral_left_degree_window = np.array(scan_ranges[int(lateral_left_start_angle*self.scans_per_degree):int(lateral_left_end_angle*self.scans_per_degree)])

#             # remove nan/inf
#             lateral_left_degree_window = lateral_left_degree_window[~np.isnan(lateral_left_degree_window)]
#             lateral_left_degree_window = lateral_left_degree_window[~np.isinf(lateral_left_degree_window)]
            
#             # left - corrected path angle distances
#             x3 = float(lateral_left_degree_window[0]) * math.cos(math.radians(theta_w + gamma_offset))
#             x4 = float(lateral_left_degree_window[-1]) * math.cos(math.radians(theta_w + gamma_offset))
#             y3 = float(lateral_left_degree_window[0]) * math.sin(math.radians(theta_w + gamma_offset))
#             y4 = float(lateral_left_degree_window[-1]) * math.sin(math.radians(theta_w + gamma_offset))
            
#             delta_x_left = x4 - x3
#             delta_y_left = y4 + y3
            
#             # Distance to current left - current wall
#             left_average_side_distance = np.mean(lateral_left_degree_window) * math.cos(math.radians(gamma_offset))
            
            
#             ############################### Future Lateral ############################### 
            
#             ######### Viewing windows : Future - right
#             future_lateral_right_start_angle = gamma_offset + self.right_degree_angle - theta_w
#             future_lateral_right_end_angle = gamma_offset + self.right_degree_angle + theta_w
#             future_lateral_right_degree_window = np.array(scan_ranges[int(future_lateral_right_start_angle*self.scans_per_degree):int(future_lateral_right_end_angle*self.scans_per_degree)])
            
#             # remove nan/inf
#             future_lateral_right_degree_window = future_lateral_right_degree_window[~np.isnan(future_lateral_right_degree_window)]
#             future_lateral_right_degree_window = future_lateral_right_degree_window[~np.isinf(future_lateral_right_degree_window)]
            
#             # Future - right path angle distances
#             future_x1 = float(future_lateral_right_degree_window[0]) * math.cos(math.radians(theta_w + gamma_offset))
#             future_x2 = float(future_lateral_right_degree_window[-1]) * math.cos(math.radians(theta_w + gamma_offset))
#             future_y1 = float(future_lateral_right_degree_window[0]) * math.sin(math.radians(theta_w + gamma_offset))
#             future_y2 = float(future_lateral_right_degree_window[-1]) * math.sin(math.radians(theta_w + gamma_offset))

#             delta_x_right = future_x2 - future_x1 
#             delta_y_right = future_y1 + future_y2
            
#             # Distance to Future - right wall
#             future_right_average_side_distance = np.mean(future_lateral_right_degree_window) * math.cos(math.radians(gamma_offset))
            
#             ######### Viewing windows : Future - left
#             # Future - delayed - left wall measurement
#             future_delayed_lateral_left_start_angle = gamma_offset + self.left_degree_angle - theta_w
#             future_delayed_lateral_left_end_angle = gamma_offset + self.left_degree_angle + theta_w
#             future_delayed_lateral_left_degree_window = np.array(scan_ranges[int(future_delayed_lateral_left_start_angle*self.scans_per_degree):int(future_delayed_lateral_left_end_angle*self.scans_per_degree)])

#             # remove nan/inf
#             future_delayed_lateral_left_degree_window = future_delayed_lateral_left_degree_window[~np.isnan(future_delayed_lateral_left_degree_window)]
#             future_delayed_lateral_left_degree_window = future_delayed_lateral_left_degree_window[~np.isinf(future_delayed_lateral_left_degree_window)]

#             # Distance to Future - delayed - left wall
#             future_delayed_left_delayed_average_side_distance = np.mean(future_delayed_lateral_left_degree_window) * math.cos(math.radians(gamma_offset))
            
#             ######### Future - left wall (Making correction)
#             phi = np.arctan((self.vx * self.tc) / future_delayed_left_delayed_average_side_distance)

#             # Viewing windows : Future - left - corrected
#             future_lateral_left_start_angle = gamma_offset + self.left_degree_angle - theta_w - phi
#             future_lateral_left_end_angle = gamma_offset + self.left_degree_angle + theta_w - phi
#             future_lateral_left_degree_window = np.array(scan_ranges[int(future_lateral_left_start_angle*self.scans_per_degree):int(future_lateral_left_end_angle*self.scans_per_degree)])

#             # remove nan/inf
#             future_lateral_left_degree_window = future_lateral_left_degree_window[~np.isnan(future_lateral_left_degree_window)]
#             future_lateral_left_degree_window = future_lateral_left_degree_window[~np.isinf(future_lateral_left_degree_window)]
            
#             # Future - left - corrected path angle distances
#             x3 = float(future_lateral_left_degree_window[0]) * math.cos(math.radians(theta_w + gamma_offset))
#             x4 = float(future_lateral_left_degree_window[-1]) * math.cos(math.radians(theta_w + gamma_offset))
#             y3 = float(future_lateral_left_degree_window[0]) * math.sin(math.radians(theta_w + gamma_offset))
#             y4 = float(future_lateral_left_degree_window[-1]) * math.sin(math.radians(theta_w + gamma_offset))
            
#             delta_x_left = x4 - x3
#             delta_y_left = y4 + y3
            
#             # Distance to future - left wall
#             future_left_average_side_distance = np.mean(lateral_left_degree_window) * math.cos(math.radians(gamma_offset))
            
#             ############################### errors for control ############################### 
            
#             right_path_heading = abs(float(np.arctan2(delta_x_right, delta_y_right)))
#             left_path_heading = abs(float(np.arctan2(delta_x_left, delta_y_left)))
#             path_difference = abs(right_path_heading - left_path_heading)
#             lateral_difference = left_average_side_distance - right_average_side_distance
#             lateral_error = self.reff_lat_wall_dist - lateral_difference
#             longitdudinal_error = -1 * min(0, (front_average_distance - self.reff_lon_wall_dist))
            
#             if path_difference > math.radians(self.path_difference_threshold):
#                 heading_error = -1 * np.sign(delta_x_right) * np.max([right_path_heading, left_path_heading])
#             else:
#                 heading_error = -1 * np.sign(delta_x_right) * np.mean([right_path_heading, left_path_heading])

#             # Publish
#             self.error_msg.data = [float(lateral_error), float(longitdudinal_error), float(heading_error)]
#             self.error_publisher.publish(self.error_msg)
#             # self.get_logger().info(f'\n'
#             #                     f'\n lateral_right_degree_window:{lateral_right_degree_window}'
#             #                     f'\n lateral_left_degree_window:{lateral_left_degree_window}'
#             #                     f'\n longitudinal_degree_window:{longitudinal_degree_window}'
#             #                     f'\n right_average_side_distance:{right_average_side_distance}'
#             #                     f'\n left_average_side_distance:{left_average_side_distance}'
#             #                     f'\n front_average_distance:{front_average_distance}'
#             #                     f'\n lateral_difference:{lateral_difference}'
#             #                     f'\n right start angle:{lateral_right_start_angle}'
#             #                     f'\n right end angle:{lateral_right_end_angle}'
#             #                     f'\n left start angle:{lateral_left_start_angle}'
#             #                     f'\n left end angle:{lateral_left_end_angle}'
#             #                     f'\n left start distance:{lateral_left_degree_window[0]}'
#             #                     f'\n left end distance:{lateral_left_degree_window[-1]}'
#             #                     f'\n right start distance:{lateral_right_degree_window[0]}'
#             #                     f'\n right end distance:{lateral_right_degree_window[-1]}'
#             #                     f'\n '
#             #                     f'\n '
#             #                     f'\n delta_x_right:{delta_x_right}'
#             #                     f'\n delta_y_right:{delta_y_right}'
#             #                     f'\n right_path_heading:{right_path_heading}'
#             #                     f'\n '
#             #                     f'\n '
#             #                     f'\n delta_x_left:{delta_x_left}'
#             #                     f'\n delta_y_left:{delta_y_left}'
#             #                     f'\n left_path_heading:{left_path_heading}'
#             #                     f'\n '
#             #                     f'\n '
#             #                     f'\n heading_error:{heading_error}'
#             #                         )
#         except IndexError or ValueError:
#             print(f"got nan or inf still or index?")
#         except:
#             print(f"Dropping Lidar message")


# def main(args=None):
#     rclpy.init(args=args)
#     tube_follower_publisher = TubeFollower()
#     try:
#         rclpy.spin(tube_follower_publisher)
#         tube_follower_publisher.destroy_node()
#         rclpy.shutdown()
#     except KeyboardInterrupt:
#         tube_follower_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
#         time.sleep(1)
#         tube_follower_publisher.destroy_node()
#         rclpy.shutdown()
#         tube_follower_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


# if __name__ == '__main__':
#     main()
