import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from depthai_sdk import OakCamera


# ROS2 Node Name
NODE_NAME = 'oakd_detection_node'
DETECTION_TOPIC_NAME = '/oakd/detections'




class OakDDetection(Node):
    def __init__(self):
        super().__init__(NODE_NAME)


        # Publisher to send object detection data
        self.detection_publisher = self.create_publisher(String, DETECTION_TOPIC_NAME, 10)


        self.last_x_center = None  # Store previous x-center
        self.prev_x_center = None  # Store older x-center for movement tracking
        self.movement_data = "NONE"  # Store movement status


        self.timer = self.create_timer(2.0, self.check_movement)  # Timer for 2-second check


        self.bridge = CvBridge()


        # Initialize Oak-D camera
        self.oak = OakCamera()
        color = self.oak.create_camera('color')


        # Define Roboflow model
        model_config = {
            'source': 'roboflow',
            'model': 'oakdlite/14',
            'key': 'OGL63eoZjIfUoLaO6Imd'  # Replace with your actual API key
        }


        # Load the model and set up callback
        self.nn = self.oak.create_nn(model_config, color)
        self.oak.callback(self.nn, self.process_detections)


        # Start camera stream
        self.oak.start(blocking=False)  # Non-blocking


        self.get_logger().info(f"{NODE_NAME} initialized successfully.")


    def process_detections(self, packet):
        """Callback function to handle object detections."""
        frame = packet.frame  # Get the raw frame


        if frame is None:
            self.get_logger().warn("No frame received.")
            return


        detections = packet.detections
        if detections:
            detection_msg = String()
            detection_data = []


            for detection in detections:
                label = detection.label_str  # Object class name
                confidence = detection.confidence  # Confidence score
                bbox = detection.bbox  # Bounding box object


                h, w, _ = frame.shape  # Get frame dimensions
                x_min, y_min = int(bbox.xmin * w), int(bbox.ymin * h)
                x_max, y_max = int(bbox.xmax * w), int(bbox.ymax * h)


                x_center = (x_min + x_max) // 2


                self.prev_x_center = self.last_x_center  
                self.last_x_center = x_center  


                # Draw bounding box
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} ({confidence*100:.2f}%)",
                            (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


                detection_data.append(f"{label} ({confidence*100:.2f}%) - "
                                      f"x_min: {x_min}, y_min: {y_min}, x_max: {x_max}, y_max: {y_max}")
               


            # Publish detection data
            detection_msg.data = f"{'; '.join(detection_data)} | Movement: {self.movement_data}"
            self.detection_publisher.publish(detection_msg)
            self.get_logger().info(f"Published: {detection_msg.data}")


        # Show the frame with bounding boxes
        cv2.imshow("Oak-D Detection", frame)
        cv2.waitKey(1)


    def check_movement(self):
        """Check if object has moved left or right after 2 seconds."""
        if self.prev_x_center is None or self.last_x_center is None:
            return  # No detection yet
           
        self.movement_data = "NONE"


        self.movement_msg = String()


        if self.last_x_center > self.prev_x_center + 10:  # Threshold for detecting right movement
            self.movement_data = "RIGHT"
        elif self.last_x_center < self.prev_x_center - 10:  # Threshold for detecting left movement
            self.movement_data = "LEFT"
        else:
            return  # No significant movement


        # Publish movement direction
       
        self.get_logger().info(f"Updated Movement: {self.movement_data}")


    def destroy(self):
        """Properly shut down the node."""
        self.oak.__exit__(None, None, None)  # Close Oak-D camera
        cv2.destroyAllWindows()
        self.get_logger().info(f"{NODE_NAME} shut down successfully.")
        super().destroy_node()




def main(args=None):
    rclpy.init(args=args)
    node = OakDDetection()


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()












