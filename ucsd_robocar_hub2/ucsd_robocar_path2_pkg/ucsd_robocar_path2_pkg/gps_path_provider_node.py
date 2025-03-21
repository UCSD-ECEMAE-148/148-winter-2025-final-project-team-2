import pandas as pd
import numpy as np
import math
import pymap3d
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

np.set_printoptions(precision=20)

class PathProvider(Node):
    def __init__(self):
      super().__init__('gps_path_provider_node')
      self.path_publisher_ = self.create_publisher(Path, '/gps_trajectory', qos_profile_sensor_data)

      # Declare ROS parameters
      self.declare_parameters(
        namespace='',
        parameters=[
          ('csv_path', "/home/projects/ros2_ws/ebu2_courtyard/ebu2_courtyard_man_2.csv"),
          ('timer_period', 0.1),
          ('frame_id', "map")
        ])

      self.csv_path = self.get_parameter('csv_path').value 
      self.timer_period = self.get_parameter('timer_period').value 
      self.frame_id = self.get_parameter('frame_id').value 

      self.path_msg = Path()
      self.path_msg.header.frame_id = self.frame_id
      self.convert_gps_to_map()
      
      # Log values used in model
      self.get_logger().info(
          f'\n csv_path: {self.csv_path}'
          f'\n timer_period: {self.timer_period}'
          f'\n frame_id: {self.frame_id}'
      )
      self.timer = self.create_timer(self.timer_period, self.publish_path)


    def convert_gps_to_map(self):
      data = np.genfromtxt(self.get_parameter("csv_path").get_parameter_value().string_value, delimiter=',', names=True, dtype='float128')
      gps_latitude = data['lat']
      gps_longitude = data['lon']
      gps_altitude = data['alt']
    #   gps_heading = data['heading']

      self.get_logger().info(f'\n From Path Provider Published (lon,lat,heading): ')
      for row in range(gps_latitude.shape[0]):
          pose_msg = PoseStamped()
          pose_msg.pose.position.x = float(gps_longitude[row])
          pose_msg.pose.position.y = float(gps_latitude[row])
          pose_msg.pose.position.z = float(gps_altitude[row])
          pose_msg.pose.orientation.x = 0.0
          pose_msg.pose.orientation.y = 0.0
          pose_msg.pose.orientation.z = 0.0
          pose_msg.pose.orientation.w = float(0.0)
          self.path_msg.poses.append(pose_msg)


    def publish_path(self):
      self.path_msg.header.stamp = self.get_clock().now().to_msg()
      self.path_msg.header.frame_id = self.frame_id
      self.path_publisher_.publish(self.path_msg)

def main(args=None):
  rclpy.init(args=args)
  path_provider = PathProvider()
  rclpy.spin(path_provider)
  path_provider.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
