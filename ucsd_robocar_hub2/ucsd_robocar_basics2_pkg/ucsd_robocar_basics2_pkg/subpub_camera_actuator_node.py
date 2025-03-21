import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Int32, Float32MultiArray, Int32MultiArray, MultiArrayDimension, MultiArrayLayout
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import time
import os


# Nodes in this program
NODE_NAME = 'subpub_camera_actuator_node'

# Topics subcribed/published
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
ACTUATOR_TOPIC_NAME = '/teleop'


class CameraActuatorSubpub(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        # adding multi-threading capability 
        self.camera_thread = MutuallyExclusiveCallbackGroup()

        # Subsciber: Camera
        self.camera_actuator_subpub = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.camera_callback, self.QUEUE_SIZE, callback_group=self.camera_thread)
        self.camera_actuator_subpub
        self.bridge = CvBridge()
        self.image_frame = None

        # Publisher: Actuator
        self.drive_pub = self.create_publisher(AckermannDriveStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()

        # setting up message structure for vesc-ackermann msg
        self.current_time = self.get_clock().now().to_msg()
        self.frame_id = 'base_link'

        # ROS parameters (setttng default values)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('live_camera_feed', 0),
                ('Ts', 0.05)
            ])

        # Update ROS parameters from config/launch file
        self.live_camera_feed = self.get_parameter('live_camera_feed').value
        self.Ts = self.get_parameter('Ts').value # controller sample time
        
        # Print ROS parameters
        self.get_logger().info(
            f'\n live_camera_feed: {self.live_camera_feed}'
            f'\n Ts: {self.Ts}'
        )

        # Call controller
        self.create_timer(self.Ts, self.controller)

    
    def camera_callback(self, data):
        # Image data
        self.image_frame = self.bridge.imgmsg_to_cv2(data)

        # display live feed
        self.live_camera_feed = self.get_parameter('live_camera_feed').value # ability to turn on/off in real-time
        if self.live_camera_feed:
            cv2.imshow('image_frame', self.image_frame)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()

    
    def controller(self):
        self.current_time = self.get_clock().now().to_msg()
        speed = 0
        delta = 0

        # publish drive control signal
        self.drive_cmd.header.stamp = self.current_time
        self.drive_cmd.header.frame_id = self.frame_id
        self.drive_cmd.drive.speed = speed
        self.drive_cmd.drive.steering_angle = delta
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
    camera_actuator_subpub = CameraActuatorSubpub()
    try:
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(camera_actuator_subpub)
        try:
            executor.spin()
        finally:
            camera_actuator_subpub.get_logger().info(f'Shutting down {NODE_NAME}...')
            camera_actuator_subpub.drive_cmd.header.stamp = camera_actuator_subpub.current_time
            camera_actuator_subpub.drive_cmd.header.frame_id = camera_actuator_subpub.frame_id
            camera_actuator_subpub.drive_cmd.drive.speed = 0.0
            camera_actuator_subpub.drive_cmd.drive.steering_angle = 0.0
            camera_actuator_subpub.drive_pub.publish(camera_actuator_subpub.drive_cmd)
            time.sleep(1)
            camera_actuator_subpub.get_logger().info(f'{NODE_NAME} shut down successfully.')
            executor.shutdown()
            camera_actuator_subpub.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()