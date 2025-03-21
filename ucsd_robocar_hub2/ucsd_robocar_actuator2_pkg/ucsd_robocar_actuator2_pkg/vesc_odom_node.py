import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from .vesc_submodule.vesc_client import VESC_

NODE_NAME = 'vesc_odom_node'
PUBLISHER_TOPIC_NAME = '/vesc_rpm'


class VescOdom(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.vesc = VESC_()
        self.motor_rpm_publisher = self.create_publisher(Int32, PUBLISHER_TOPIC_NAME, 10)
        self.motor_rpm = Float32()

        self.default_K_rpm = 1.5
        self.default_K_rpm_offset = -125.88
        self.default_K_v = 4921.82
        self.default_K_v_offset = 93.59
        self.declare_parameters(
            namespace='',
            parameters=[
                ('K_rpm', self.default_K_rpm),
                ('K_v', self.default_K_rpm),
                ('K_rpm', self.default_K_v),
                ('K_v_offset', self.default_K_v_offset)
            ])
        self.K_rpm = float(self.get_parameter('K_rpm').value)
        self.K_rpm_offset = float(self.get_parameter('K_rpm_offset').value)

        # publish a message every 0.1 seconds
        self.timer_period = 1 / 30 
        self.timer = self.create_timer(self.timer_period, self.get_vesc_rpm)  # Create the timer

    def get_vesc_rpm(self):
        motor_rpm_raw = int(self.vesc.get_rpm())
        motor_rpm_est = self.K_rpm * motor_rpm_raw + self.K_rpm_offset

        self.motor_rpm.data = int(self.vesc.get_rpm())
        self.motor_rpm_publisher.publish()

def main(args=None):
    rclpy.init(args=args)
    try:
        vesc_odom = VescOdom()
        rclpy.spin(vesc_odom)
        vesc_odom.destroy_node()
        rclpy.shutdown()
    except:
        vesc_odom.get_logger().info(f'Could not connect to VESC, Shutting down {NODE_NAME}...')
        vesc_odom.destroy_node()
        rclpy.shutdown()
        vesc_odom.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
