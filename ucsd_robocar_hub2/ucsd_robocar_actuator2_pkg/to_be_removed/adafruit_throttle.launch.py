import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'ucsd_robocar_actuator2_pkg'
    throttle_node_name = 'adafruit_throttle_node'

    original_topic_name = '/throttle'
    new_topic_name = LaunchConfiguration('topic_name', default=original_topic_name)

    ld = LaunchDescription()
    throttle_node = Node(
        package=pkg,
        executable=throttle_node_name,
        output='screen',
        remappings=[(original_topic_name,new_topic_name)]
    )
    ld.add_action(throttle_node)
    return ld
