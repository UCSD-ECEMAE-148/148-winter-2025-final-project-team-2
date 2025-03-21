#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit
import board
import busio

NODE_NAME = 'adafruit_servo_node'
TOPIC_NAME = '/servo'

'''
[0, 180]degrees: [full right, full left]
'''

class AdafruitServo(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.servo_subscriber = self.create_subscription(Float32, TOPIC_NAME, self.send_values_to_adafruit, 10)
        self.default_bus_num = int(1)
        self.default_servo_channel = int(3)
        self.default_max_limit = 170
        self.default_min_limit = 10
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bus_num', self.default_bus_num),
                ('servo_channel', self.default_servo_channel),
                ('max_limit', self.default_max_limit),
                ('min_limit', self.default_min_limit)
            ])
        self.bus_num = int(self.get_parameter('bus_num').value)
        self.servo_channel = int(self.get_parameter('servo_channel').value)
        self.max_limit = int(self.get_parameter('max_limit').value)
        self.min_limit = int(self.get_parameter('min_limit').value)

        if self.bus_num == 0:
            i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
            self.kit = ServoKit(channels=16, i2c=i2c_bus0)
        else:
            self.kit = ServoKit(channels=16)

    def send_values_to_adafruit(self, msg):
        servo_angle_raw = msg.data
        servo_angle = self.clamp(servo_angle_raw, self.max_limit, self.min_limit)
        self.kit.servo[self.servo_channel].angle = servo_angle

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
    adafruit_servo = AdafruitServo()
    try:
        rclpy.spin(adafruit_servo)
        adafruit_servo.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        adafruit_servo.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        adafruit_servo.destroy_node()
        rclpy.shutdown()
        adafruit_servo.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
