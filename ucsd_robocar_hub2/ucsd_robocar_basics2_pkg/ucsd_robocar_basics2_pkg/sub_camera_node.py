import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import time
import os


# Nodes in this program
NODE_NAME = 'sub_camera_node'

# Topics subcribed
CAMERA_TOPIC_NAME = '/camera/color/image_raw'


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        # adding multi-threading capability 
        self.camera_thread = MutuallyExclusiveCallbackGroup()

        # Subsciber: Camera
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.camera_callback, self.QUEUE_SIZE, callback_group=self.camera_thread)
        self.camera_subscriber
        self.bridge = CvBridge()
        self.image_frame = None

        # ROS parameters (setttng default values)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('live_camera_feed', 0)
            ])

        # Update ROS parameters from config/launch file
        self.live_camera_feed = self.get_parameter('live_camera_feed').value
        
        # Print ROS parameters
        self.get_logger().info(
            f'\n live_camera_feed: {self.live_camera_feed}'
        )

    
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


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    try:
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(camera_subscriber)
        try:
            executor.spin()
        finally:
            camera_subscriber.get_logger().info(f'Shutting down {NODE_NAME}...')
            # Kill cv2 windows and node
            cv2.destroyAllWindows()
            camera_subscriber.get_logger().info(f'{NODE_NAME} shut down successfully.')
            executor.shutdown()
            camera_subscriber.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()