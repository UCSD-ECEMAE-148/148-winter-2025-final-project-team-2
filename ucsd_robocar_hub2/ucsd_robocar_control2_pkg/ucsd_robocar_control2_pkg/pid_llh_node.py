import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from ackermann_msgs.msg import AckermannDriveStamped
import time
import math
import os
import numpy as np

NODE_NAME = 'pid_llh_node'
ERROR_TOPIC_NAME = '/error'
ACTUATOR_TOPIC_NAME = '/teleop'


class PidController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        # Actuator control
        self.drive_pub = self.create_publisher(AckermannDriveStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()

        # Error subscriber
        self.error_subscriber = self.create_subscription(Float32MultiArray, ERROR_TOPIC_NAME, self.error_measurement, self.QUEUE_SIZE)
        self.error_subscriber

        # setting up message structure for vesc-ackermann msg
        self.current_time = self.get_clock().now().to_msg()
        self.frame_id = 'base_link'

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('integral_max', 0),
                ('heading_upper_error_threshold', 0.15),
                ('heading_lower_error_threshold', 0.15),
                ('long_upper_error_threshold', 0.15),
                ('long_lower_error_threshold', 0.15),
                ('zero_speed', 0.0),
                ('max_speed', 5),
                ('min_speed', 0.1),
                ('max_right_steering', 0.4),
                ('max_left_steering', -0.4),
                ('Ts', 0.05)
            ])
        self.Kp = self.get_parameter('Kp_steering').value
        self.Ki = self.get_parameter('Ki_steering').value
        self.Kd = self.get_parameter('Kd_steering').value
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

        # initializing PID control
        self.e_y_buffer = 0
        self.e_x_buffer = 0
        self.e_theta_buffer = 0
        self.e_y = 0
        self.e_y_1 = 0
        self.e_x = 0
        self.e_theta = 0

        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        
        self.get_logger().info(
            f'\n Kp_steering: {self.Kp}'
            f'\n Ki_steering: {self.Ki}'
            f'\n Kd_steering: {self.Kd}'
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
        )
        # Call controller
        self.create_timer(self.Ts, self.controller)

    def error_measurement(self, error_data):
        error_data_check = np.array([error_data.data[0], error_data.data[1], error_data.data[2]])
        if not (np.isnan(error_data_check).any()):
            self.e_y_buffer = error_data.data[0]
            self.e_x_buffer = error_data.data[1]
            self.e_theta_buffer = error_data.data[2]

    def get_latest_measurements(self):
        self.e_y = self.e_y_buffer
        self.e_x = self.e_x_buffer
        self.e_theta = self.e_theta_buffer
        self.current_time = self.get_clock().now().to_msg()

    def controller(self):
        # Get latest measurement
        self.get_latest_measurements()

        # Steering PID terms
        self.proportional_error = self.Kp * self.e_y
        self.derivative_error = self.Kd * (self.e_y - self.e_y_1) / self.Ts
        self.integral_error += self.Ki * self.e_y * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        delta_raw = self.proportional_error + self.derivative_error + self.integral_error

        # Throttle gain scheduling (function of error)
        self.inf_throttle = self.min_speed - ((self.min_speed - self.max_speed) / (self.heading_upper_error_threshold - self.heading_lower_error_threshold)) * self.heading_upper_error_threshold
        speed_raw = ((self.min_speed - self.max_speed) / (self.heading_upper_error_threshold - self.heading_lower_error_threshold)) * abs(self.e_theta) + self.inf_throttle

        # clamp values
        delta = self.clamp(delta_raw, self.max_right_steering, self.max_left_steering)
        speed = self.clamp(speed_raw, self.max_speed, self.min_speed)
        
        self.get_logger().info(f'\n'
                               f'\n ex:{self.e_x}'
                               f'\n ey:{self.e_y}'
                               f'\n e_theta:{self.e_theta}'
                               f'\n delta:{delta_raw}'
                               f'\n speed:{speed_raw}'
                               f'\n clamped delta:{delta}'
                               f'\n clamped speed:{speed}'
                               )
        self.e_y_1 = self.e_y

        if self.e_x < self.long_upper_error_threshold:
            # Publish values
            try:
                # publish drive control signal
                self.drive_cmd.header.stamp = self.current_time
                self.drive_cmd.header.frame_id = self.frame_id
                self.drive_cmd.drive.speed = speed
                self.drive_cmd.drive.steering_angle = -delta
                self.drive_pub.publish(self.drive_cmd)

            except KeyboardInterrupt:
                self.drive_cmd.header.stamp = self.current_time
                self.drive_cmd.header.frame_id = self.frame_id
                self.drive_cmd.drive.speed = 0
                self.drive_cmd.drive.steering_angle = 0
                self.drive_pub.publish(self.drive_cmd)
        else:
            # publish drive control signal
            self.drive_cmd.header.stamp = self.current_time
            self.drive_cmd.header.frame_id = self.frame_id
            self.drive_cmd.drive.speed = self.zero_speed
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


def main(args=None):
    rclpy.init(args=args)
    pid_publisher = PidController()
    try:
        rclpy.spin(pid_publisher)
        pid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        pid_publisher.drive_cmd.header.stamp = pid_publisher.current_time
        pid_publisher.drive_cmd.header.frame_id = pid_publisher.frame_id
        pid_publisher.drive_cmd.drive.speed = 0.0
        pid_publisher.drive_cmd.drive.steering_angle = 0.0
        pid_publisher.drive_pub.publish(pid_publisher.drive_cmd)
        time.sleep(1)
        pid_publisher.destroy_node()
        rclpy.shutdown()
        pid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
