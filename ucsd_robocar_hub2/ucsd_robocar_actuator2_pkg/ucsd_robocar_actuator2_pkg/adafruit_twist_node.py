import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit
import board
import busio
import time

NODE_NAME = 'adafruit_twist_node'
TOPIC_NAME = '/cmd_vel'

class AdafruitTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.send_values_to_adafruit, 10)

        # Default board values
        self.default_bus_num = int(1)
        self.default_steering_channel = int(1)
        self.default_throttle_channel = int(2)
        self.default_steering_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_throttle_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_max_right_steering = 0.90
        self.default_straight_steering = 0.05
        self.default_max_left_steering = 0.-85
        self.default_max_throttle = 0.2
        self.default_zero_throttle = 0.0
        self.default_min_throttle = -0.2
        
        # declaring ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', self.default_bus_num),
                ('steering_channel', self.default_steering_channel),
                ('throttle_channel', self.default_throttle_channel),
                ('steering_polarity', self.default_steering_polarity),
                ('throttle_polarity', self.default_throttle_polarity),
                ('max_right_steering', self.default_max_right_steering),
                ('straight_steering', self.default_straight_steering),
                ('max_left_steering', self.default_max_left_steering),
                ('max_throttle', self.default_max_throttle),
                ('zero_throttle', self.default_zero_throttle),
                ('min_throttle', self.default_min_throttle)
            ])

        # Get ROS parameters from config
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.steering_channel = int(self.get_parameter('steering_channel').value)
        self.throttle_channel = int(self.get_parameter('throttle_channel').value)
        self.steering_polarity = int(self.get_parameter('steering_polarity').value)
        self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.straight_steering = self.get_parameter('straight_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value
        self.max_right_steering_angle = self.remap(self.max_right_steering)
        self.max_left_steering_angle = self.remap(self.max_left_steering)
        self.steering_offset = self.remap(self.straight_steering)
        self.zero_throttle = self.get_parameter('zero_throttle').value # between [-1,1] but should be around 0
        self.max_throttle = self.get_parameter('max_throttle').value # between [-1,1]
        self.min_throttle = self.get_parameter('min_throttle').value # between [-1,1]

        # Set BUS ID
        if self.bus_num == 0:
            i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
            self.kit = ServoKit(channels=16, i2c=i2c_bus0)
        else:
            self.kit = ServoKit(channels=16)
        
        self.get_logger().info(
            f'\nbus_num: {self.bus_num}'
            f'\nsteering_channel: {self.steering_channel}'
            f'\nthrottle_channel: {self.throttle_channel}'
            f'\nsteering_polarity: {self.steering_polarity}'
            f'\nthrottle_polarity: {self.throttle_polarity}'
            f'\n(max_right_steering, max_right_steering_angle): ({self.max_right_steering}, {self.max_right_steering_angle})'
            f'\n(straight_steering, steering_offset): ({self.straight_steering}, {self.steering_offset})'
            f'\n(max_left_steering, max_left_steering_angle): ({self.max_left_steering}, {self.max_left_steering_angle})'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            )

    def send_values_to_adafruit(self, msg):
        # Steering map from [-1,1] --> [-90, 90] : [max_left,max_right] # to do: implement into calibration
        steering_angle_raw = float(self.steering_offset + self.remap(msg.angular.z))
        steering_angle = self.clamp(steering_angle_raw, self.max_right_steering_angle, self.max_left_steering_angle)
        throttle_float = self.clamp(msg.linear.x, self.max_throttle, self.min_throttle)

        # Send values to adafruit board 
        self.kit.servo[self.steering_channel].angle = float(90 + self.steering_polarity * steering_angle)
        self.kit.continuous_servo[self.throttle_channel].throttle = self.throttle_polarity * throttle_float

    def remap(self, value):
        input_start = -1
        input_end = 1
        output_start = -90
        output_end = 90
        normalized_output = float(output_start + (value - input_start) * ((output_end - output_start) / (input_end - input_start)))
        return normalized_output

    def clamp(self, data, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if data < lower_bound:
            data_c = lower_bound
        elif data > upper_bound:
            data_c = upper_bound
        else:
            data_c = data
        return data_c


def main(args=None):
    rclpy.init(args=args)
    try:
        adafruit_twist = AdafruitTwist()
        rclpy.spin(adafruit_twist)
        adafruit_twist.destroy_node()
        rclpy.shutdown()
    except:
        adafruit_twist.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        adafruit_twist.kit.continuous_servo[adafruit_twist.throttle_channel].throttle = adafruit_twist.throttle_polarity * adafruit_twist.zero_throttle
        time.sleep(1)
        adafruit_twist.destroy_node()
        rclpy.shutdown()
        adafruit_twist.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
