#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
import time
import math
import os
import numpy as np

NODE_NAME = 'pid_gps_node'
ERROR_TOPIC_NAME = '/error'
ACTUATOR_TOPIC_NAME = '/drive'


class PidController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        # threads
        self.error_thread = MutuallyExclusiveCallbackGroup()

        # Actuator control
        # self.drive_pub = self.create_publisher(TwistStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        # self.drive_cmd = TwistStamped()
        # self.drive_pub = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        # self.drive_cmd = Twist()

        self.drive_pub = self.create_publisher(AckermannDriveStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()

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
                ('max_right_steering', -0.4),
                ('max_left_steering', 0.4),
                ('Ts', 0.1),
                ('delta_rate_max', 0.015),
                ('speed_rate_max', 0.1),
                ('pid_calibration_config_location', '/test.yaml'),
                ('pid_config_location', '/test.yaml'),
                ('show_logger', 1)
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
        self.delta_rate_max = self.get_parameter('delta_rate_max').value # max change in steering angle /second (rad/s)
        self.speed_rate_max = self.get_parameter('speed_rate_max').value # max change in speed (m/s)
        self.show_logger = self.get_parameter('show_logger').value # controller sample time
        
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

        self.delta_prev = 0
        self.speed_prev = 0
        # self.delta_rate_max = math.radians(1)
        
        self.get_logger().info(
            f'\n Kp_lat: {self.Kp_lat}'
            f'\n Ki_lat: {self.Ki_lat}'
            f'\n Kd_lat: {self.Kd_lat}'
            f'\n Kp_head: {self.Kp_head}'
            f'\n Ki_head: {self.Ki_head}'
            f'\n Kd_head: {self.Kd_head}'
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
            f'\n delta_rate_max: {self.delta_rate_max}'
            f'\n speed_rate_max: {self.speed_rate_max}'
        )
        # Call controller
        self.create_timer(self.Ts, self.controller)

    def error_measurement(self, error_data):
        error_data_check = np.array([error_data.data[0], error_data.data[1], error_data.data[2], error_data.data[3]])
        if not (np.isnan(error_data_check).any()):
            self.e_x_buffer = error_data.data[0]
            self.e_y_buffer = error_data.data[1]
            self.e_theta_buffer = error_data.data[2]
            self.path_complete_percent_buffer = error_data.data[3]

    def get_latest_measurements(self):
        self.e_x = self.e_x_buffer
        self.e_y = self.e_y_buffer
        self.e_theta = self.e_theta_buffer
        self.path_complete_percent = self.path_complete_percent_buffer
        self.current_time = self.get_clock().now().to_msg()
        self.Kp_lat = self.get_parameter('Kp_lateral').value
        self.Ki_lat = self.get_parameter('Ki_lateral').value
        self.Kd_lat = self.get_parameter('Kd_lateral').value
        self.Kp_head = self.get_parameter('Kp_heading').value
        self.Ki_head = self.get_parameter('Ki_heading').value
        self.Kd_head = self.get_parameter('Kd_heading').value
        self.integral_max = self.get_parameter('integral_max').value
        self.delta_rate_max = self.get_parameter('delta_rate_max').value
        self.speed_rate_max = self.get_parameter('speed_rate_max').value

    def calculate_pid(self, kp, ki, kd, error_current, error_previous, integral_error):
        proportional_error = kp * error_current
        derivative_error = kd * (error_current - error_previous) / self.Ts
        integral_error += ki * error_current * self.Ts
        integral_error = self.clamp(integral_error, self.integral_max)
        control_raw = proportional_error + derivative_error + integral_error
        return control_raw, integral_error
    
    def calculate_error_map(self, error, limit_upper, limit_lower):
        b = self.min_speed - ((self.min_speed - self.max_speed) / (limit_upper - limit_lower)) * limit_upper
        m = ((self.min_speed - self.max_speed) / (limit_upper - limit_lower))
        y = m * abs(error) + b
        return y

    def controller(self):
        # Get latest measurement
        self.get_latest_measurements()

        # Steering PID terms
        control_error_lat, self.integral_error_lat = self.calculate_pid(self.Kp_lat, self.Ki_lat, self.Kd_lat, self.e_y, self.e_y_1, self.integral_error_lat)
        control_error_head, self.integral_error_head = self.calculate_pid(self.Kp_head, self.Ki_head, self.Kd_head, self.e_theta, self.e_theta_1, self.integral_error_head)
        delta_raw = control_error_lat + control_error_head

        # Throttle gain scheduling (function of error)
        speed_raw = self.calculate_error_map(self.e_y, self.long_upper_error_threshold, self.long_lower_error_threshold)

        # constrain limits
        delta_limit_clamp = self.clamp(delta_raw, self.max_left_steering, self.max_right_steering)
        speed_limit_clamp = self.clamp(speed_raw, self.max_speed, self.min_speed)

        # constrain rates
        delta_rate_clamp = self.steer_rate_clamp(delta_limit_clamp)
        speed_rate_clamp = self.speed_rate_clamp(speed_limit_clamp)

        delta = delta_limit_clamp
        speed = speed_rate_clamp
        
        if self.show_logger:
            self.get_logger().info(
                f'\n'
                f'\n errors:'
                f'\n     ex (lon): {self.e_x}'
                f'\n     ey (lat): {self.e_y}'
                f'\n     e_theta (degrees): {math.degrees(self.e_theta)}'
                f'\n raw:'
                f'\n     speed_raw (m/s): {speed_raw}'
                f'\n     delta_raw (degrees): {math.degrees(delta_raw)}'
                f'\n limits clamped:'
                f'\n     speed_limit_clamp (m/s): {speed_limit_clamp}'
                f'\n     delta_limit_clamp (degrees): {math.degrees(delta_limit_clamp)}'
                f'\n rates clamped:'
                f'\n     delta_rate_clamp (degrees): {math.degrees(delta_rate_clamp)}'
                f'\n     speed_rate_clamp (m/s): {speed_rate_clamp}'
                f'\n published:'
                f'\n     delta (degrees): {math.degrees(delta)}'
                f'\n     speed (m/s): {speed}'
                )
        
        self.e_y_1 = self.e_y
        self.e_theta_1 = self.e_theta
        self.delta_prev = delta
        self.speed_prev = speed

        if self.path_complete_percent < 100.0:
            # Publish values
            try:
                # publish drive control signal
                self.drive_cmd.header.stamp = self.current_time
                self.drive_cmd.header.frame_id = self.frame_id
                self.drive_cmd.drive.speed = speed
                self.drive_cmd.drive.steering_angle = delta
                self.drive_pub.publish(self.drive_cmd)

            except KeyboardInterrupt:
                self.drive_cmd.header.stamp = self.current_time
                self.drive_cmd.header.frame_id = self.frame_id
                self.drive_cmd.drive.speed = 0.0
                self.drive_cmd.drive.steering_angle = 0.0
                self.drive_pub.publish(self.drive_cmd)
        else:
            # publish drive control signal
            self.drive_cmd.header.stamp = self.current_time
            self.drive_cmd.header.frame_id = self.frame_id
            self.drive_cmd.drive.speed = 0.0
            self.drive_cmd.drive.steering_angle = 0.0
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

    def steer_rate_clamp(self, current_delta):
        delta_change = current_delta - self.delta_prev
        if abs(delta_change) > (self.delta_rate_max * self.Ts):
            delta_clamp = self.delta_prev + np.sign(delta_change) * self.delta_rate_max * self.Ts
        else:
            delta_clamp = current_delta
        return delta_clamp

    def speed_rate_clamp(self, current_speed):
        speed_change = current_speed - self.speed_prev
        if abs(speed_change) > (self.speed_rate_max * self.Ts):
            speed_clamp = min(current_speed, self.speed_prev +  np.sign(speed_change) * self.speed_rate_max * self.Ts)
        else:
            speed_clamp = current_speed
        return speed_clamp


def main(args=None):
    rclpy.init(args=args)
    pid_publisher = PidController()
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
    try:
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(pid_publisher)
        try:
            executor.spin()
        finally:
            pid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
            # pid_publisher.drive_cmd.linear.x = 0.0
            # pid_publisher.drive_cmd.angular.z = 0.0
            # pid_publisher.drive_pub.publish(pid_publisher.drive_cmd)

            pid_publisher.drive_cmd.header.stamp = pid_publisher.current_time
            pid_publisher.drive_cmd.header.frame_id = pid_publisher.frame_id
            pid_publisher.drive_cmd.drive.speed = 0.0
            pid_publisher.drive_cmd.drive.steering_angle = 0.0
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
