#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, Twist
import time
import math
import os
import numpy as np
import cv2


PID_CALIBRATION_NODE_NAME = 'pid_gps_calibration_node'
PID_NODE_NAME = 'pid_gps_node'
ERROR_TOPIC_NAME = '/error'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

def callback(x):
    pass

def create_track_bars(window_name):
    Kp_max = 100
    Kp_default = 0
    Ki_max = 100
    Ki_default = 0
    Kd_max = 100
    Kd_default = 0
    integral_max = 100
    integral_default = 0

    cv2.namedWindow(window_name)

    image = np.zeros((500, 500, 3), np.uint8)
    cv2.imshow(window_name, image)

    # PID parameters
    cv2.createTrackbar('Kp_lat', window_name, Kp_default, Kp_max, callback) # percent
    cv2.createTrackbar('Ki_lat', window_name, Ki_default, Ki_max, callback) # percent
    cv2.createTrackbar('Kd_lat', window_name, Kd_default, Kd_max, callback) # percent
    cv2.createTrackbar('Kp_head', window_name, Kp_default, Kp_max, callback) # percent
    cv2.createTrackbar('Ki_head', window_name, Ki_default, Ki_max, callback) # percent
    cv2.createTrackbar('Kd_head', window_name, Kd_default, Kd_max, callback) # percent
    cv2.createTrackbar('integral_max', window_name, integral_default, integral_max, callback) # percent

    # Test motor control
    cv2.createTrackbar('test_motor_control', window_name, 0, 1, callback)

    # save values
    cv2.createTrackbar('save_parameters', window_name, 0, 1, callback)


class PidController(Node):
    def __init__(self):
        super().__init__(PID_CALIBRATION_NODE_NAME)
        self.QUEUE_SIZE = 10

        # threads
        self.error_thread = MutuallyExclusiveCallbackGroup()

        # Actuator control
        self.drive_pub = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = Twist()

        # Error subscriber
        self.error_subscriber = self.create_subscription(
            Float32MultiArray, 
            ERROR_TOPIC_NAME, 
            self.error_measurement, 
            self.QUEUE_SIZE, 
            callback_group=self.error_thread)
        self.error_subscriber

        # setting up message structure for twist msg
        self.current_time = self.get_clock().now().to_msg()
        self.frame_id = 'base_link'

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_lateral', 1.0),
                ('Ki_lateral', 0.0),
                ('Kd_lateral', 0.0),
                ('Kp_heading', 1.0),
                ('Ki_heading', 0.0),
                ('Kd_heading', 0.0),
                ('integral_max', 0.0),
                ('heading_upper_error_threshold', 0.0),
                ('heading_lower_error_threshold', 0.0),
                ('long_upper_error_threshold', 0.0),
                ('long_lower_error_threshold', 0.0),
                ('zero_speed', 0.0),
                ('max_speed', 5.0),
                ('min_speed', 0.1),
                ('max_right_steering', 0.4),
                ('max_left_steering', -0.4),
                ('Ts', 0.1),
                ('pid_calibration_config_location', '/test.yaml'),
                ('pid_config_location', '/test.yaml')
            ])
        
        self.Kp_lat = self.get_parameter('Kp_lateral').value
        self.Ki_lat = self.get_parameter('Ki_lateral').value
        self.Kd_lat = self.get_parameter('Kd_lateral').value
        self.Kp_head = self.get_parameter('Kp_heading').value
        self.Ki_head = self.get_parameter('Ki_heading').value
        self.Kd_head = self.get_parameter('Kd_heading').value
        
        self.integral_max = self.get_parameter('integral_max').value 
        self.heading_upper_error_threshold = self.get_parameter('heading_upper_error_threshold').value # between [0,1]
        self.heading_lower_error_threshold = self.get_parameter('heading_lower_error_threshold').value # between [0,1]
        self.long_upper_error_threshold = self.get_parameter('long_upper_error_threshold').value # between [0,1]
        self.long_lower_error_threshold = self.get_parameter('long_lower_error_threshold').value # between [0,1]
        
        self.zero_speed = self.get_parameter('zero_speed').value  # should be around 0
        self.max_speed = self.get_parameter('max_speed').value  # between [0,5] m/s
        self.min_speed = self.get_parameter('min_speed').value  # between [0,5] m/s 
        self.max_right_steering = self.get_parameter('max_right_steering').value  # negative(max_left) 
        self.max_left_steering = self.get_parameter('max_left_steering').value  # between abs([0,0.436332]) radians (0-25degrees)
        self.Ts = self.get_parameter('Ts').value # controller sample time

        self.pid_config_location = self.get_parameter('pid_config_location').value # controller sample time
        self.pid_calibration_config_location = self.get_parameter('pid_calibration_config_location').value # controller sample time

        # initializing PID control
        self.e_y_buffer = 0
        self.e_x_buffer = 0
        self.e_theta_buffer = 0
        self.path_complete_percent_buffer = 0
        self.e_y = 0
        self.e_y_1 = 0
        self.e_theta_1 = 0
        self.e_x = 0
        self.e_theta = 0
        self.path_complete_percent=0

        self.proportional_error_y = 0 # proportional error term for steering
        self.derivative_error_y = 0 # derivative error term for steering
        self.integral_error_lat = 0 # integral error term for steering
        self.integral_error_head = 0
        
        self.get_logger().info(
            f'\n Kp_lat: {self.Kp_lat}'
            f'\n Ki_lat: {self.Ki_lat}'
            f'\n Kd_lat: {self.Kd_lat}'
            f'\n Kp_head: {self.Kp_head}'
            f'\n Ki_head: {self.Ki_head}'
            f'\n Kd_head: {self.Kd_head}'
            f'\n integral_max: {self.integral_max}'
            f'\n heading_upper_error_threshold: {self.heading_upper_error_threshold}'
            f'\n heading_lower_error_threshold: {self.heading_lower_error_threshold}'
            f'\n long_upper_error_threshold: {self.long_upper_error_threshold}'
            f'\n long_lower_error_threshold: {self.long_lower_error_threshold}'
            f'\n zero_speed: {self.zero_speed}'
            f'\n max_speed: {self.max_speed}'
            f'\n min_speed: {self.min_speed}'
            f'\n max_right_steering: {self.max_right_steering}'
            f'\n max_left_steering: {self.max_left_steering}'
            f'\n Ts: {self.Ts}'
            f'\n pid_config_location: {self.pid_config_location}'
            f'\n pid_calibration_config_location: {self.pid_calibration_config_location}'
        )

        self.THR_STR_WINDOW_NAME = 'throttle_and_steering'
        try:
            cv2.setTrackbarPos('Kp_lat', self.THR_STR_WINDOW_NAME, self.Kp_lat)
            cv2.setTrackbarPos('Ki_lat', self.THR_STR_WINDOW_NAME, self.Ki_lat)
            cv2.setTrackbarPos('Kd_lat', self.THR_STR_WINDOW_NAME, self.Kd_lat)
            cv2.setTrackbarPos('Kp_head', self.THR_STR_WINDOW_NAME, self.Kp_head)
            cv2.setTrackbarPos('Ki_head', self.THR_STR_WINDOW_NAME, self.Ki_head)
            cv2.setTrackbarPos('Kd_head', self.THR_STR_WINDOW_NAME, self.Kd_head)
            cv2.setTrackbarPos('integral_max', self.THR_STR_WINDOW_NAME, self.steering_polarity)
        except:
            pass

        # self.create_info_window()

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.75
        self.font_thickness = 1
        self.image = np.zeros((250, 575, 3), np.uint8)

        # Call controller
        self.create_timer(self.Ts, self.controller)

    # def create_info_window(self):
    #     # create blank image
    #     self.black_image = np.zeros((300, 500, 3), np.uint8)

    #     # Calculate the maximum width of the variable names and values
    #     self.max_name_width = max(len(name) for name, _ in vars)
    #     self.max_value_width = max(len(value) for _, value in vars)

    #     # Calculate the width and height of each column
    #     self.num_columns = 2
    #     self.column_width = self.max_name_width * 10 + self.max_value_width * 10
    #     self.column_height = 30

    #     # Calculate the width and height of the image
    #     self.num_rows = (len(vars) + self.num_columns - 1) // self.num_columns
    #     self.image_width = self.num_columns * self.column_width
    #     self.image_height = self.num_rows * self.column_height

    #     # Create a white image to use as the background
    #     self.image = np.ones((self.image_height, self.image_width, 3), np.uint8) * 255


    def error_measurement(self, error_data):
        error_data_check = np.array([error_data.data[0], error_data.data[1], error_data.data[2], error_data.data[3]])
        if not (np.isnan(error_data_check).any()):
            self.e_x_buffer = error_data.data[0]
            self.e_y_buffer = error_data.data[1]
            self.e_theta_buffer = error_data.data[2]
            self.path_complete_percent_buffer = error_data.data[3]

    def get_latest_measurements(self):
        self.e_y = self.e_y_buffer
        self.e_x = self.e_x_buffer
        self.e_theta = self.e_theta_buffer
        self.path_complete_percent = self.path_complete_percent_buffer
        self.current_time = self.get_clock().now().to_msg()
        self.Kp_lat = self.get_parameter('Kp_lateral').value
        self.Ki_lat = self.get_parameter('Ki_lateral').value
        self.Kd_lat = self.get_parameter('Kd_lateral').value
        self.Kp_head = self.get_parameter('Kp_heading').value
        self.Ki_head = self.get_parameter('Ki_heading').value
        self.Kd_head = self.get_parameter('Kd_heading').value

    def calculate_pid(self, kp, ki, kd, error_current, error_previous, integral_error):
        proportional_error = kp * error_current
        derivative_error = kd * (error_current - error_previous) / self.Ts
        integral_error += ki * error_current * self.Ts
        integral_error = self.clamp(integral_error, self.integral_max)
        control_raw = proportional_error + derivative_error + integral_error
        return control_raw, integral_error
    
    def calculate_error_map(self, error, limit_upper, limit_lower):
        inf_throttle = self.min_speed - ((self.min_speed - self.max_speed) / (limit_upper - limit_lower)) * limit_upper
        error_map_speed = ((self.min_speed - self.max_speed) / (limit_upper - limit_lower)) * abs(error) + inf_throttle
        return error_map_speed

    def controller(self):
        # Get latest measurement
        self.get_latest_measurements()

        # Get slider bar values --- Motor parameters
        self.Kp_lat = float(cv2.getTrackbarPos('Kp_lat', self.THR_STR_WINDOW_NAME)/100) # percent to decimal
        self.Ki_lat = float(cv2.getTrackbarPos('Ki_lat', self.THR_STR_WINDOW_NAME)/100) # percent to decimal
        self.Kd_lat = float(cv2.getTrackbarPos('Kd_lat', self.THR_STR_WINDOW_NAME)/100) # percent to decimal
        self.Kp_head = float(cv2.getTrackbarPos('Kp_head', self.THR_STR_WINDOW_NAME)/100) # percent to decimal
        self.Ki_head = float(cv2.getTrackbarPos('Ki_head', self.THR_STR_WINDOW_NAME)/100) # percent to decimal
        self.Kd_head = float(cv2.getTrackbarPos('Kd_head', self.THR_STR_WINDOW_NAME)/100) # percent to decimal
        self.integral_max = float(cv2.getTrackbarPos('integral_max', self.THR_STR_WINDOW_NAME)/100) # percent to decimal
        test_motor_control = int(cv2.getTrackbarPos('test_motor_control', self.THR_STR_WINDOW_NAME))
        save_parameters = int(cv2.getTrackbarPos('save_parameters', self.THR_STR_WINDOW_NAME))

        # Steering PID terms
        control_error_lat, self.integral_error_lat = self.calculate_pid(self.Kp_lat, self.Ki_lat, self.Kd_lat, self.e_y, self.e_y_1, self.integral_error_lat)
        control_error_head, self.integral_error_head = self.calculate_pid(self.Kp_head, self.Ki_head, self.Kd_head, self.e_theta, self.e_theta_1, self.integral_error_head)
        delta_raw = control_error_lat + control_error_head

        # Throttle gain scheduling (function of error)
        speed_raw_heading = self.calculate_error_map(self.e_theta, self.heading_upper_error_threshold, self.heading_lower_error_threshold)
        speed_raw_long = self.calculate_error_map(self.e_x, self.long_upper_error_threshold, self.long_lower_error_threshold)

        # clamp values
        delta = self.clamp(delta_raw, self.max_right_steering, self.max_left_steering)
        # speed = self.clamp(speed_raw, self.max_speed, self.min_speed)
        speed = 1.0
        
        self.e_y_1 = self.e_y
        self.e_theta_1 = self.e_theta
        
        # publish control message if slider bar is true
        if test_motor_control:
            if self.path_complete_percent < 100.0:
                # Publish values
                try:
                    # publish drive control signal    
                    self.drive_cmd.linear.x = speed   
                    self.drive_cmd.angular.z = delta       
                    self.drive_pub.publish(self.drive_cmd)

                except KeyboardInterrupt:
                    self.drive_cmd.linear.x = 0
                    self.drive_cmd.angular.z = 0
                    self.drive_pub.publish(self.drive_cmd)
            else:
                # publish drive control signal
                self.drive_cmd.linear.x = 0.0
                self.drive_cmd.angular.z = 0.0
                self.drive_pub.publish(self.drive_cmd)
        else:
            pass

        self.get_logger().info(
            f'\n'
            f'\n errors:'
            f'\n     ex (lon): {self.e_x}'
            f'\n     ey (lat): {self.e_y}'
            f'\n     e_theta (degrees): {math.degrees(self.e_theta)}'
            f'\n raw:'
            f'\n     speed_raw_heading (m/s): {speed_raw_heading}'
            f'\n     speed_raw_long (m/s): {speed_raw_long}'
            f'\n     delta_raw (degrees): {math.degrees(delta_raw)}'
            f'\n clamped:'
            f'\n     clamped speed (m/s): {speed}'
            f'\n     clamped delta (degrees): {math.degrees(delta)}'
            )

        # update display
        
        # var_info = [
        #     ("errors:", " "),
        #     ("    ex (lon):", self.e_x),
        #     ("    ey (lat):", self.e_y),
        #     ("    e_theta (heading):", self.e_theta),
        #     ("raw:", " "),
        #     ("    speed_raw_heading (m/s):", speed_raw_heading),
        #     ("    speed_raw_long (m/s):", speed_raw_long),
        #     ("    delta_raw (rad):", delta_raw),
        #     ("clamped:", " "),
        #     ("    clamped speed (m/s):", speed),
        #     ("    clamped delta (m/s):", delta),
        # ]
        

        # turn right -- (0:-0.8] 
        # turn left ++ (0:1.5]

        # error -- path to the right
        # error ++ path to the left


        # x = 50
        # y = 50
        # line_height = 30
        # for (name, value) in var_info:
        #     cv2.putText(self.image, f"{name} {value}", (x, y), self.font, self.font_scale, (255, 255, 255), 1)
        #     # cv2.putText(self.image, f"test", (50, 50), self.font, self.font_scale, (255, 255, 255), 1)
        #     y += line_height

        # # cv2.putText(self.image, "test", (50, 50), self.font, self.font_scale, (0, 0, 0), 1)
        # # Show the image in a window
        # cv2.imshow("Variables", self.image)
        # cv2.imshow(THR_STR_WINDOW_NAME)
        # cv2.getWindowProperty()
        # cv2.imshow('throttle_and_steering', pid_publisher.image)
        cv2.waitKey(1)
        

        # save parameters
        if save_parameters:
            self.save_parameters(PID_CALIBRATION_NODE_NAME, self.pid_calibration_config_location)
            self.save_parameters(PID_NODE_NAME, self.pid_config_location)

    def save_parameters(self, node_name, file_name):
        f = open(file_name, "w")
        f.write(
            f"{node_name}: \n"
            f"  ros__parameters: \n"
            f"    Kp_lat : {self.Kp_lat} \n"
            f"    Ki_lat : {self.Ki_lat} \n"
            f"    Kd_lat : {self.Kd_lat} \n"
            f"    Kp_head : {self.Kp_head} \n"
            f"    Ki_head : {self.Ki_head} \n"
            f"    Kd_head : {self.Kd_head} \n"
            f"    integral_max : {self.integral_max} \n"
            f"    zero_speed : {self.zero_speed} \n"
            f"    max_speed : {self.max_speed} \n"
            f"    min_speed : {self.min_speed} \n"
            f"    heading_upper_error_threshold : {self.heading_upper_error_threshold} \n"
            f"    heading_lower_error_threshold : {self.heading_lower_error_threshold} \n"
            f"    long_upper_error_threshold : {self.long_upper_error_threshold} \n"
            f"    long_lower_error_threshold : {self.long_lower_error_threshold} \n"
            f"    max_right_steering : {self.max_right_steering} \n"
            f"    max_left_steering : {self.max_left_steering} \n"
            f"    Ts : {self.Ts} \n"
            f'    pid_calibration_config_location : "{self.pid_calibration_config_location}" \n'
            f'    pid_config_location : "{self.pid_config_location}" \n'
        )
        f.close()



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
    pid_publisher = PidController()
    create_track_bars(pid_publisher.THR_STR_WINDOW_NAME)
    steer_right_range_deg = f"(0:-{round(math.degrees(0.8),2)}]"
    steer_left_range_deg = f"(0:{round(math.degrees(1.5),2)}]"
    var_info = [
            ("Steering Ranges (degrees):", " "),
            ("    Turn RIGHT (-):", steer_right_range_deg),
            ("    Turn LEFT (+):", steer_left_range_deg),
            ("Error Meaninings (m):", " "),
            ("    Lateral ey (-):", "Path is to the RIGHT"),
            ("    Lateral ey (+):", "Path is to the LEFT")
        ]
    x = 50
    y = 50
    line_height = 30
    for (name, value) in var_info:
        cv2.putText(pid_publisher.image, f"{name} {value}", (x, y), pid_publisher.font, pid_publisher.font_scale, (255, 255, 255), 2)
        y += line_height
    try:
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(pid_publisher)
        while True:
            cv2.imshow('throttle_and_steering', pid_publisher.image)
            cv2.waitKey(1)
            try:
                executor.spin()
            finally:
                pid_publisher.get_logger().info(f'Shutting down {PID_CALIBRATION_NODE_NAME}...')
                cv2.destroyAllWindows()
                pid_publisher.drive_cmd.linear.x = 0.0
                pid_publisher.drive_cmd.angular.z = 0.0
                pid_publisher.drive_pub.publish(pid_publisher.drive_cmd)
                time.sleep(1)
                executor.shutdown()
                pid_publisher.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
