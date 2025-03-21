import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ROS2 Node Name
NODE_NAME = 'oakd_detection_subscriber'
DETECTION_TOPIC_NAME = '/oakd/detections'


class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Subscriber to receive detection results from Oak-D Lite
        self.subscription = self.create_subscription(
            String,
            DETECTION_TOPIC_NAME,
            self.detection_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info(f"{NODE_NAME} initialized and listening on {DETECTION_TOPIC_NAME}")

    def detection_callback(self, msg):
        """Processes incoming detection messages."""
        self.get_logger().info(f"Received Detection: {msg.data}")

        # Optional: Process the detections further
        detections = msg.data.split("; ")  # Split multiple detections
        for detection in detections:
            print(f"Parsed Detection: {detection}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down detection subscriber...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



