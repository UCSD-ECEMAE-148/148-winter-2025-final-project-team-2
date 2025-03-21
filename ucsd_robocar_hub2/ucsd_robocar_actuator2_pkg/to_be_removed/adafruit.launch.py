import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'ucsd_robocar_actuator2_pkg'
    steering_node_name = 'adafruit_steering_node'
    throttle_node_name = 'adafruit_throttle_node'

    ld = LaunchDescription()

    steering_node = Node(
        package=pkg,
        executable=steering_node_name,
        output='screen'
        )

    throttle_node = Node(
        package=pkg,
        executable=throttle_node_name,
        output='screen'
        )

    ld.add_action(steering_node)
    ld.add_action(throttle_node)
    return ld
