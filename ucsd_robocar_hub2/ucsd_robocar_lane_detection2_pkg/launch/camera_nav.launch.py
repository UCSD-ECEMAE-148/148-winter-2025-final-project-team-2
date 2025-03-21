import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml


def generate_launch_description():
    lane_detection2_package = 'ucsd_robocar_lane_detection2_pkg'
    calibration_file = 'ros_racer_calibration.yaml'
    ld_node_name = 'lane_detection_node'
    lg_node_name = 'lane_guidance_node'

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(lane_detection2_package),
        'config',
        calibration_file)

    lane_detection_node = Node(
        package=lane_detection2_package,
        executable=ld_node_name,
        output='screen',
        parameters=[config])

    lane_guidance_node = Node(
        package=lane_detection2_package,
        executable=lg_node_name,
        output='screen',
        parameters=[config])

    ld.add_action(lane_detection_node)
    ld.add_action(lane_guidance_node)
    return ld
