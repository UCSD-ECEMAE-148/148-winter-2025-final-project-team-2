import time
import os
import rclpy
from rclpy.node import Node 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from .controller_submodule.lqr_calculator import LQRDesign
from .controller_submodule.car_model import CarModel
import numpy as np
import pandas as pd
import math
import time

NODE_NAME = 'lqr_node'

# ACTUATOR_TOPIC_NAME = '/vesc/high_level/ackermann_cmd_mux/output'
# ACTUATOR_TOPIC_NAME = '/teleop'
ACTUATOR_TOPIC_NAME = '/lqr_controller_test'
JOY_TOPIC_NAME = '/teleop'

# PATH_TOPIC_NAME = '/global_trajectory'
POSE_TOPIC_NAME = '/slam_out_pose'
PATH_TOPIC_NAME = '/trajectory'
# ODOM_TOPIC_NAME = '/odom'

class LqrController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.frame_id = 'base_link'
        self.QUEUE_SIZE = 10
        self.data_file = "/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_control2_pkg/data/test.csv"
        self.df = pd.DataFrame(columns = ['time', 'joy_delta', 'joy_speed', 'lqr_delta', 'lqr_speed'])
         
        # self.controller_thread = MutuallyExclusiveCallbackGroup()
        self.path_thread = MutuallyExclusiveCallbackGroup()
        # self.odom_thread = MutuallyExclusiveCallbackGroup()
        self.pose_thread = MutuallyExclusiveCallbackGroup()
        self.joy_thread = MutuallyExclusiveCallbackGroup()

        # Actuator control
        self.drive_pub = self.create_publisher(AckermannDriveStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()
        
        ### Get sensor measurements ###

        # # Get Odometry measurements
        # self.odom_subscriber = self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odom_measurement, 10, callback_group=self.odom_thread)
        # self.odom_subscriber

        # Get GPS/Lidar measurements
        self.pose_subscriber = self.create_subscription(PoseStamped, POSE_TOPIC_NAME, self.pose_measurement, self.QUEUE_SIZE, callback_group=self.pose_thread)
        self.pose_subscriber

        # Get Reference Trajectory
        self.path_subscriber = self.create_subscription(Path, PATH_TOPIC_NAME, self.set_path, rclpy.qos.qos_profile_sensor_data, callback_group=self.path_thread)
        self.path_subscriber

        # Get Joystick commands
        self.joy_subscriber = self.create_subscription(AckermannDriveStamped, JOY_TOPIC_NAME, self.set_joy_command, self.QUEUE_SIZE, callback_group=self.joy_thread)
        self.joy_subscriber

        self.start_time = time.time()
        self.current_time = self.get_clock().now().to_msg()

        # Sensor measurements
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.yaw_rate = 0
        self.vx = 0.1
        self.vy = 0
        self.joy_speed = 0
        self.joy_steering = 0

        # Sensor measurements Buffer
        self.x_buffer = 0
        self.y_buffer = 0
        self.yaw_buffer = 0
        self.yaw_rate_buffer = 0
        self.vx_buffer = 0.1
        self.vy_buffer = 0
        self.joy_speed_buffer = 0
        self.joy_steering_buffer = 0

        # Path coordinates
        self.x_path = []
        self.y_path = []
        self.yaw_path = []
        self.line_error_threshold = 0.001 # (m)

        # Controller modules
        self.car_model = CarModel()
        self.lqr_calc = LQRDesign(self.car_model)
        self.x0 = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.state_measurement = self.x0
        self.sys = self.car_model.build_error_model(self.vx)

        # Calculated states
        self.ecg = 0  # cross-track error
        self.theta_e = 0  # heading error
        self.theta_e_dot = 0  # heading error yaw_rate
        self.delta_raw = 0

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('k1_gain', 1.0),
                ('k2_gain', 1.0),
                ('k3_gain', 1.0),
                ('k4_gain', 1.0),
                ('k1_coeff', [1.0, 1.0, 1.0]),
                ('k2_coeff', [1.0, 1.0, 1.0]),
                ('k3_coeff', [1.0, 1.0, 1.0]),
                ('k4_coeff', [1.0, 1.0, 1.0]),
                ('error_threshold', 0.15),
                ('zero_speed', 0.0),
                ('max_speed', 5),
                ('min_speed', 0.1),
                ('max_right_steering', 0.4),
                ('max_left_steering', -0.4)
            ])

        self.k1_gain=self.get_parameter('k1_gain').value
        self.k2_gain=self.get_parameter('k2_gain').value
        self.k3_gain=self.get_parameter('k3_gain').value
        self.k4_gain=self.get_parameter('k4_gain').value
        self.k1_coeff=self.get_parameter('k1_coeff').value
        self.k2_coeff=self.get_parameter('k2_coeff').value
        self.k3_coeff=self.get_parameter('k3_coeff').value
        self.k4_coeff=self.get_parameter('k4_coeff').value
        self.error_threshold=self.get_parameter('error_threshold').value  # between [0,1]
        self.zero_speed=self.get_parameter('zero_speed').value  # should be around 0
        self.max_speed=self.get_parameter('max_speed').value  # between [0,5] m/s
        self.min_speed=self.get_parameter('min_speed').value  # between [0,5] m/s 
        self.max_right_steering=self.get_parameter('max_right_steering').value  # negative(max_left) 
        self.max_left_steering=self.get_parameter('max_left_steering').value  # between abs([0,0.436332]) radians (0-25degrees)

        self.get_logger().info(
            f'\nk1_gain: {self.k1_gain}'
            f'\nk2_gain: {self.k2_gain}'
            f'\nk3_gain: {self.k3_gain}'
            f'\nk4_gain: {self.k4_gain}'
            f'\nk1_coeff: {self.k1_coeff}'
            f'\nk2_coeff: {self.k2_coeff}'
            f'\nk3_coeff: {self.k3_coeff}'
            f'\nk4_coeff: {self.k4_coeff}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_speed: {self.zero_speed}'
            f'\nmax_speed: {self.max_speed}'
            f'\nmin_speed: {self.min_speed}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
            f'\nself.state_measurement: {self.state_measurement}'
            f'\ntype(self.state_measurement): {type(self.state_measurement)}'
            f'\nshape(self.state_measurement): {self.state_measurement.shape}'
        )

        # Call controller
        self.Ts = 0.01  # contoller sample time
        self.create_timer(self.Ts, self.controller)
        self.create_timer(1.0, self.save_csv)

    def odom_measurement(self, odom_data):
        # TODO: what is frequency of data coming in?

        # car position
        self.x_buffer = odom_data.pose.pose.position.x
        self.y_buffer = odom_data.pose.pose.position.y

        # car orientation
        # FIXME: confirm coordinate axes
        quaternion = (odom_data.orientation.x, odom_data.orientation.y, odom_data.orientation.z, odom_data.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw_buffer = euler[2]

        # car velocity
        self.vx_buffer = odom_data.twist.twist.linear.x
        self.vy_buffer = odom_data.twist.twist.linear.y

    def pose_measurement(self, pose_data):
        # car orientation
        quaternion = (pose_data.pose.orientation.x, pose_data.pose.orientation.y, pose_data.pose.orientation.z, pose_data.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw_buffer = euler[2]

        # car coordinates
        self.x_buffer = pose_data.pose.position.x
        self.y_buffer = pose_data.pose.position.y
        # self.get_logger().info(f"Updating POSE (x): ({self.x})")

    def set_path(self, path_data):
        # path coordinates (GLOBAL)
        self.x_path = np.array([pose.pose.position.x for pose in path_data.poses])
        self.y_path = np.array([pose.pose.position.y for pose in path_data.poses])
        self.get_logger().info(f"first val PATH (x): ({self.x_path[0]})")

    def set_joy_command(self, joy_data):
        self.joy_speed_buffer = joy_data.drive.speed
        self.joy_steering_buffer = joy_data.drive.steering_angle

    def get_cross_track_error(self):
        # self.get_logger().info("Updating CROSS-TRACK-ERROR")
        
        # find 2 closest points in path with car
        error_x = self.x_path - self.x
        error_y = self.y_path - self.y
        error_mag = np.power(np.power(error_x,2) + np.power(error_y, 2), 0.5)
        error_mag1, error_mag2 = np.partition(error_mag, 1)[0:2]
        error_mag1_index = np.argwhere(error_mag == error_mag1)[0][0]
        error_mag2_index = np.argwhere(error_mag == error_mag2)[0][0]
        Px1 = self.x_path[error_mag1_index]
        Px2 = self.x_path[error_mag2_index]
        Py1 = self.y_path[error_mag1_index]
        Py2 = self.y_path[error_mag2_index]
        
        # create line extrapolations to determine cross-track error
        # (threshold added to account for zero/infinite slopes)
        
        # path line
        delta_x = Px2 - Px1 + self.line_error_threshold
        delta_y = Py2 - Py1 + self.line_error_threshold
        theta_path = float(np.arctan2(delta_y, delta_x))
        path_slope = delta_y / delta_x
        path_intercept = Py1 - path_slope * Px1
        
        # car line
        car_slope = -1 / path_slope
        car_intercept = self.y - car_slope * self.x
        ecg_x = (path_intercept - car_intercept) / (car_slope - path_slope)
        ecg_y = car_slope * ecg_x + car_intercept

        # get actual cross-track error distance and use sign of slope to determine direction
        ecg_r = np.power(np.power((self.x - ecg_x),2) + np.power((self.y - ecg_y), 2), 0.5)
        e_cg_sign = np.sign(car_slope)
        e_cg = float(e_cg_sign * ecg_r)
        self.get_logger().info(f"{e_cg},{theta_path}")
        return e_cg, theta_path

    def get_latest_measurements(self):
        # car orientation
        self.yaw = self.yaw_buffer
        
        # car coordinates
        self.x = self.x_buffer
        self.y = self.y_buffer

        # car speed
        # self.vx = self.vx_buffer
        self.vy = self.vy_buffer

        # manual control
        self.vx = self.joy_speed_buffer
        self.joy_speed = self.joy_speed_buffer 
        self.joy_steering = self.joy_steering_buffer

        # time
        self.current_time = self.get_clock().now().to_msg()

    def update_gains(self):
        K = self.lqr_calc.compute_single_gain_sample(self.sys)
        return K

    def update_states(self):
        # self.get_logger().info("Updating STATES")
        theta_e_km1 = self.state_measurement[2][0]
        e_cg, theta_path = self.get_cross_track_error()
        theta_e_k = theta_path - self.yaw
        self.state_measurement[0][0] = e_cg
        self.state_measurement[1][0] = self.vy + self.vx * math.sin(theta_e_k)
        self.state_measurement[2][0] = theta_e_k
        self.state_measurement[3][0] = (theta_e_k - theta_e_km1) / self.Ts
        self.get_logger().info(f"states: {self.state_measurement}")

    def controller(self):
        """
        Need:
        -pose data
        -path data
        -vx (measured longitudinal velocity)

        states:
        -ecg (cross-trackk error from center of gravity (cg))
        -ecg_dot (cross-trackk error rate from cg)
        -theta_e (heading error)
        -theta_e_dot (heading error rate)

        inputs:
        -delta (steering angle)

        published message:
        -steering_angle (radians)
        -speed (m/s)
        """

        # Update Car model LTV system --- A(Vx)
        self.get_latest_measurements()
        self.sys = self.car_model.build_error_model(self.vx)

        # get updated gains
        K = self.update_gains()

        # Steering LQR
        self.delta_raw = -np.dot(K[0], self.state_measurement).flat[0]

        # Throttle gain scheduling
        tracking_error = self.state_measurement[0][0]
        self.inf_throttle = self.min_speed - (self.min_speed - self.max_speed) / (1 - self.error_threshold)
        speed_raw = ((self.min_speed - self.max_speed) / (1 - self.error_threshold)) * abs(tracking_error) + self.inf_throttle

        # Clamp control inputs
        delta = self.clamp(self.delta_raw, self.max_right_steering, self.max_left_steering)
        speed = self.clamp(speed_raw, self.max_speed, self.min_speed)

        # Publish values
        self.current_time
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
            self.drive_cmd.drive.speed = 0
            self.drive_cmd.drive.steering_angle = 0
            self.drive_pub.publish(self.drive_cmd)

        # Update States
        self.update_states()

        # write out
        self.compare_manual_and_lqr()
        

    def compare_manual_and_lqr(self):
        self.df = pd.concat([self.df, pd.DataFrame.from_records([{'time': time.time() - self.start_time, 'joy_delta': self.joy_steering, 'joy_speed': self.joy_speed, 'lqr_delta': self.delta_raw, 'lqr_speed': self.drive_cmd.drive.speed}])])

    def save_csv(self):
        self.df.to_csv(self.data_file, index = False)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound == None:
            lower_bound=-upper_bound  # making lower bound symmetric about zero
        if value < lower_bound:
            value_c=lower_bound
        elif value > upper_bound:
            value_c=upper_bound
        else:
            value_c=value
        return value_c


def main(args=None):
    rclpy.init(args=args)
    lqr_publisher=LqrController()
    try:
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(lqr_publisher)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            lqr_publisher.save_csv()
            lqr_publisher.destroy_node()
    except KeyboardInterrupt:
        lqr_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        lqr_publisher.drive_cmd.header.stamp = lqr_publisher.current_time
        lqr_publisher.drive_cmd.header.frame_id = lqr_publisher.frame_id
        lqr_publisher.drive_cmd.drive.speed = 0.0
        lqr_publisher.drive_cmd.drive.steering_angle = 0.0
        lqr_publisher.drive_pub.publish(lqr_publisher.drive_cmd)
        time.sleep(1)
    finally:
        lqr_publisher.save_csv()
        lqr_publisher.get_logger().info(f'{NODE_NAME} saved data to {lqr_publisher.data_file}.')
        lqr_publisher.destroy_node()
        rclpy.shutdown()
        lqr_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
