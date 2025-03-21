import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
import os.path
from .vesc_submodule.vesc_client import VESC_
from std_msgs.msg import Float32


NODE_NAME = 'vesc_steering_node'
STEERING_TOPIC_NAME = '/steering'


v = VESC_()

class VescSteering(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.steering_subscriber = self.create_subscription(Float32, STEERING_TOPIC_NAME, self.callback, 10)


    def callback(self, data):
        vesc_min_limit = 0
        vesc_max_limit = 1
        data_min_limit = -1
        data_max_limit = 1

        # mapping from [-1,1] --> [0,1]    
        steering_angle = float(-0.1 + ((data.data-data_min_limit)*(vesc_max_limit - vesc_min_limit))/(data_max_limit-data_min_limit))
        v.send_servo_angle(steering_angle)


def main(args=None):
    rclpy.init(args=args)
    vesc_steering = VescSteering()
    rclpy.spin(vesc_steering)
    vesc_steering.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
