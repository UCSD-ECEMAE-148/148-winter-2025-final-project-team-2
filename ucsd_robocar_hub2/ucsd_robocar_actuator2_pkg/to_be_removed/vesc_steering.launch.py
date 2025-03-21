import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'ucsd_robocar_actuator2_pkg'
    steering_node_name = 'vesc_steering_node'

    original_topic_name = '/steering'
    new_topic_name = LaunchConfiguration('topic_name', default=original_topic_name)

    ld = LaunchDescription()
    steering_node = Node(
        package=pkg,
        executable=steering_node_name,
        output='screen',
        remappings=[(original_topic_name,new_topic_name)]
    )
    ld.add_action(steering_node)
    return ld
