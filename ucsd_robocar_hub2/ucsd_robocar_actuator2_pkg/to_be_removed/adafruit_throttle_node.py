#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

NODE_NAME = 'adafruit_throttle_node'
TOPIC_NAME = '/throttle'

'''
[-1,1] : [max_reverse, max_forward]
'''


class AdafruitThrottle(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rpm_subscriber = self.create_subscription(Float32, TOPIC_NAME, self.callback, 10)
        kit = ServoKit(channels=16)


    def callback(self, data):
        normalized_throttle = data.data
        kit.continuous_servo[2].throttle = normalized_throttle


def main(args=None):
    rclpy.init(args=args)
    adafruit_throttle = AdafruitThrottle()
    rclpy.spin(adafruit_throttle)
    adafruit_throttle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
