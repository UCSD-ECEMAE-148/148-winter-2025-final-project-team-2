import os
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from .controller_submodule.lqr_calculator import LQRDesign
from .controller_submodule.car_model import CarModel
from .controller_submodule.linear_kalman_filter import LinearKalmanFilter
import numpy as np

NODE_NAME = 'lqg_node'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

POSE_TOPIC_NAME = '/amcl_pose'
PATH_TOPIC_NAME = '/global_trajectory'
ERROR_TOPIC_NAME = '/path_error'
IMU_TOPIC_NAME = '/imu'
ODOM_TOPIC_NAME = '/odom'


class LqgController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        ### Get sensor measurements ###
        #
        # Get IMU measurement
        self.velocity_subscriber = self.create_subscription(Imu, IMU_TOPIC_NAME, self.imu_measurement, 10)
        self.velocity_subscriber

        # Get GPS/Lidar measurements
        self.pose_subscriber = self.create_subscription(Pose, POSE_TOPIC_NAME, self.pose_measurement, 10)
        self.pose_subscriber

        # Get Odometry measurements
        self.odom_subscriber = self.create_subscription(Odom, ODOM_TOPIC_NAME, self.odom_measurement, 10)
        self.odom_subscriber

        # Get Reference Trajectory
        self.path_subscriber = self.create_subscription(Path, PATH_TOPIC_NAME, self.set_path, 10)
        self.path_subscriber

        # TODO: Make compatible with Camera Navigation via: Lane detection
        # Get road marker error measurements from camera
        # self.pose_error_subscriber = self.create_subscription(Float32MultiArray, ERROR_TOPIC_NAME, self.camera_measurment, 10)
        # self.pose_error_subscriber

        ### Controller and State Estimate modules ###
        self.car_model = CarModel()
        self.lqr_calc = LQRDesign(self.car_model)
        self.kalman_calc = LinearKalmanFilter()
        self.P = np.diag([1, 1, 1, 1])
        self.Qo = np.diag([1, 1, 1, 1])
        self.Ro = [0.1]
        self.x0 = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.state_measurement = self.x0
        self.state_est = self.x0

        # Sensor measurements
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.roll_rate = 0
        self.pitch_rate = 0
        self.yaw_rate = 0
        self.vx = 0
        self.vy = 0
        self.ax = 0
        self.ay = 0
        self.az = 0

        # Path coordinates
        self.x_path = []
        self.y_path = []
        self.z_path = []
        self.roll_path = []
        self.pitch_path = []
        self.yaw_path = []

        # Calculated states
        self.ecg = 0  # cross-track error
        self.theta_e = 0  # heading error
        self.theta_e_dot = 0  # heading error rate

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('error_threshold', 0.15),
                ('zero_throttle', 0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0)
            ])
        self.error_threshold = self.get_parameter('error_threshold').value  # between [0,1]
        self.zero_throttle = self.get_parameter('zero_throttle').value  # between [-1,1] but should be around 0
        self.max_throttle = self.get_parameter('max_throttle').value  # between [-1,1]
        self.min_throttle = self.get_parameter('min_throttle').value  # between [-1,1]
        self.max_right_steering = self.get_parameter('max_right_steering').value  # between [-1,1]
        self.max_left_steering = self.get_parameter('max_left_steering').value  # between [-1,1]

        self.get_logger().info(
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )

        # Call controller
        self.Ts = 1/100  # contoller publish frequency (Hz)
        self.create_timer(self.Ts, self.controller)

    def imu_measurement(self, imu_data):
        self.get_logger().info("Updating IMU")

        # TODO: what is frequency of data coming in?
        
        # FIXME: confirm coordinate axes
        # orientation
        quaternion = (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw_imu = euler[2]

        # angular velocity
        self.roll_rate = imu_data.angular_velocity.x
        self.pitch_rate = imu_data.angular_velocity.y
        self.yaw_rate = imu_data.angular_velocity.z

        # linear acceleration
        self.ax = imu_data.linear_acceleration.x
        self.ay = imu_data.linear_acceleration.y
        self.az = imu_data.linear_acceleration.z

        # linear velocity
        self.vx = self.vx + (self.ax * self.Ts)
        self.vy = self.vy + (self.ay * self.Ts)

    def odom_measurement(self, odom_data):
        # position
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y
        self.z = odom_data.pose.pose.position.z

        # FIXME: confirm coordinate axes
        # orientation
        quaternion = (odom_data.orientation.x, odom_data.orientation.y, odom_data.orientation.z, odom_data.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

        # velocity
        self.vx = odom_data.twist.twist.linear.x
        self.vy = odom_data.twist.twist.linear.y

    def pose_measurement(self, pose_data):
        self.get_logger().info("Updating POSE")

        # TODO: what is frequency of data coming in?
        # FIXME: confirm coordinate axes
        # car coordinates
        self.x = pose_data.position.x
        self.y = pose_data.position.y
        self.z = pose_data.position.z

    def set_path(self, path_data):
        self.get_logger().info("Updating PATH")

        # TODO: Currently not working with Lidar Nav
        # FIXME: confirm coordinate axes
        # path orientation (Currently not working with Lidar Nav)
        # quaternion = (path_data.poses[0].pose.orientation.x, path_data.poses[0].pose.orientation.y,
        #               path_data.poses[0].pose.orientation.z, path_data.poses[0].pose.orientation.w)
        # euler = euler_from_quaternion(quaternion)
        # self.roll_path = euler[0]
        # self.pitch_path = euler[1]
        # self.yaw_path = euler[2]

        # path coordinates (GLOBAL)
        self.x_path = path_data.poses[0].pose.position.x
        self.y_path = path_data.poses[0].pose.position.y
        self.theta_path = np.arctan(self.y_path, self.x_path)

    def calc_cross_track_error(self):
        efa_x = self.x_path - self.x
        efa_y = self.y_path - self.y
        efa_mag = np.power(np.power(efa_x,2) + np.power(efa_y, 2), 0.5);
        efa_mag1, efa_mag2 = np.partition(efa_mag, 1)[0:2]
        efa_mag1_index = np.where(efa_mag == efa_mag1)
        efa_mag2_index = np.where(efa_mag == efa_mag2)
        Px1 = self.x_path[efa_mag1_index]
        Px2 = self.x_path[efa_mag2_index]
        Py1 = self.y_path[efa_mag1_index]
        Py2 = self.y_path[efa_mag2_index]
        delta_x = Px2 - Px1
        delta_y = Py2 - Py1
        R_x = self.x - Px1
        R_y = self.y - Py1
        r_2 = np.power(delta_x, 2) + np.power(delta_y, 2)
        e_cg = (R_y * delta_x - R_x * delta_y) / r_2
        return e_cg, e_cg_index

    def update_states(self):
        self.get_logger().info("Updating STATES")
        delta_x_path = self.x_path[1] - self.x_path[0]
        delta_y_path = self.y_path[1] - self.y_path[0]
        pose_error_x = self.x - self.x_path[0]
        pose_error_y = self.y - self.y_path[0]
        theta_e_km1 = self.state_measurement[0][2]
        e_cg, e_cg_index = self.calc_cross_track_error()
        theta_e_k = self.theta_path[e_cg_index] - self.yaw_imu
        self.state_measurement[0][0] = e_cg
        self.state_measurement[0][1] = self.vy + self.vx * math.sin(theta_e_k)
        self.state_measurement[0][2] = theta_e_k
        self.state_measurement[0][3] = (theta_e_k - theta_e_km1) / self.Ts

    def controller(self):
        self.get_logger().info("Updating CONTROLLER")
        """
        sensor measurements:
        -imu, odom and pose data

        reference tracking:
        -path data

        states:
        x1 - ecg: cross-trackk error from center of gravity (cg) --- = (pose_error_y * delta_x_path - pose_error_x * delta_y_path) / (delta_x_path^2 + delta_y_path^2)
        x2 - ecg_dot: cross-trackk error rate from cg --- = vy + vx * sin(theta_error)
        x3 - theta_e: heading error --- = path_angle - car_yaw_angle
        x4 - theta_e_dot: heading error rate --- = (theta_e_k - theta_e_km1) / self.Ts # theta_e_k = heading error at sample k AND theta_e_km1 = heading error at sample k - 1
        """
        self.get_logger().info("Here 1")

        # Update Car model LTV system --- A(Vx)
        sys = self.car_model.build_error_model(self.vx)

        # Get gains
        K = self.lqr_calc.compute_single_gain_sample(sys)

        # Steering LQR
        self.get_logger().info("Here 2")
        steering_float_raw = -np.dot(K[0], self.state_est).flat[0]

        # TODO: function of cross-track error (e_cg) or heading error (theta_e)??
        # Throttle gain scheduling
        tracking_error = self.state_measurement[0][0]
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle) / (1 - self.error_threshold)) * abs(tracking_error) + self.inf_throttle

        # Clamp control inputs
        # FIXME: need to convert to radians and m/s respectively 
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Get Current Measurement
        self.y = self.car_model.calc_output(self.state_measurement)

        # Get optimal state estimates
        self.state_est, self.P = self.kalman_calc.lkf(sys, self.state_est, steering_float, self.y, self.P, self.Qo, self.Ro)
        self.get_logger().info("Here 3")

        # Publish values
        try:
            # publish control_submodule signals
            self.twist_cmd.angular.z = steering_float
            self.twist_cmd.linear.x = throttle_float
            self.twist_publisher.publish(self.twist_cmd)
        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)
        self.get_logger().info("Here 4")

        # Get new sensor measurements
        self.update_states()

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound is None:
            lower_bound = -upper_bound  # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c


def main(args=None):
    rclpy.init(args=args)
    lqg_publisher = LqgController()
    try:
        rclpy.spin(lqg_publisher)
        lqg_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        lqg_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        lqg_publisher.twist_cmd.linear.x = lqg_publisher.zero_throttle
        lqg_publisher.twist_publisher.publish(lqg_publisher.twist_cmd)
        time.sleep(1)
        lqg_publisher.destroy_node()
        rclpy.shutdown()
        lqg_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
