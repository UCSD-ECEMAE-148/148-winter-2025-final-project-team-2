from ast import arg
from turtle import pos
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathProvider(Node):
    def __init__(self):
        super().__init__('path_provider_node')
        self.path_publisher_ = self.create_publisher(Path, '/trajectory', qos_profile_sensor_data)
        self.declare_parameter("csv_path",'/home/projects/ros2_ws/ebu2_courtyard/ebu2_courtyard_man_2.csv')
        self.declare_parameter("timer", 0.1)
        self.declare_parameter("frame_id", 'map')
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.load_path()
        self.timer = self.create_timer(self.get_parameter("timer").get_parameter_value().double_value, self.publish_path)
    
    def load_path(self):
        df = pd.read_csv(self.get_parameter("csv_path").get_parameter_value().string_value)
        df = df.dropna(how='any')
        for _, row in df.iterrows():
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = row['px']
            pose_msg.pose.position.y = row['py']
            pose_msg.pose.position.z = row['pz']
            pose_msg.pose.orientation.x = row['qx']
            pose_msg.pose.orientation.y = row['qy']
            pose_msg.pose.orientation.z = row['qz']
            pose_msg.pose.orientation.w = row['qw']
            self.path_msg.poses.append(pose_msg)
    
    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher_.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    path_provider = PathProvider()
    rclpy.spin(path_provider)
    path_provider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
