#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit
import board
import busio

NODE_NAME = 'adafruit_continuous_servo_node'
TOPIC_NAME = '/continuous_servo'

'''
[-1, 1] : [max reverse, max forward]
'''

class AdafruitContinuousServo(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.continuous_servo_subscriber = self.create_subscription(Float32, TOPIC_NAME, self.send_values_to_adafruit, 10)
        self.default_bus_num = int(1)
        self.default_continuous_servo_channel = int(4)
        self.default_max_forward_limit = 0.2
        self.default_max_reverse_limit = -0.2
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', self.default_bus_num),
                ('continuous_servo_channel', self.default_continuous_servo_channel),
                ('max_forward', self.default_max_forward_limit),
                ('max_reverse', self.default_max_reverse_limit)
            ])
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.continuous_servo_channel = int(self.get_parameter('continuous_servo_channel').value)
        self.max_forward = int(self.get_parameter('max_forward').value)
        self.max_reverse = int(self.get_parameter('max_reverse').value)

        if self.bus_num == 0:
            i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
            self.kit = ServoKit(channels=16, i2c=i2c_bus0)
        else:
            self.kit = ServoKit(channels=16)

    def send_values_to_adafruit(self, msg):
        continuous_servo_throttle_raw = msg.data
        continuous_servo_throttle = self.clamp(continuous_servo_throttle_raw, self.max_forward, self.max_reverse)
        self.kit.continuous_servo[self.continuous_servo_channel].throttle = continuous_servo_throttle

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
    adafruit_continuous_servo = AdafruitContinuousServo()
    try:
        rclpy.spin(adafruit_continuous_servo)
        adafruit_continuous_servo.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        adafruit_continuous_servo.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        adafruit_continuous_servo.destroy_node()
        rclpy.shutdown()
        adafruit_continuous_servo.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()

