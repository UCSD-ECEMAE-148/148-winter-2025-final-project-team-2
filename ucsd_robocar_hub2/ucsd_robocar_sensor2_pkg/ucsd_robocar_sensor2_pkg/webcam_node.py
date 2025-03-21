import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

CAMERA_NODE_NAME = 'webcam_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'


class ImagePublisher(Node):
    def __init__(self):
        super().__init__(CAMERA_NODE_NAME)
        self.camera_publisher = self.create_publisher(Image, CAMERA_TOPIC_NAME, 10)
        self.camera_publisher
        self.bridge = CvBridge()
        # publish a message every 0.1 seconds
        self.timer_period = 1 / 30 
        self.timer = self.create_timer(self.timer_period, self.live_cam_feed)  # Create the timer
        self.cap = cv2.VideoCapture(0)  # Create a VideoCapture object
        self.bridge = CvBridge()

    def live_cam_feed(self):
        ret, frame = self.cap.read()
          
        if ret == True:
            self.camera_publisher.publish(self.bridge.cv2_to_imgmsg(frame))
            # Display the message on the console
            # self.get_logger().info('Publishing image')


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
        image_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print(f"\nShutting down {CAMERA_NODE_NAME}...")
        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        image_publisher.destroy_node()
        rclpy.shutdown()
        print(f"{CAMERA_NODE_NAME} shut down successfully.")


if __name__ == '__main__':
    main()
