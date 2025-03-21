import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace



def generate_launch_description():
    pkg = 'ucsd_robocar_actuator2_pkg'
    lane_detection2_package = 'ucsd_robocar_lane_detection2_pkg'
    node_name = 'adafruit_twist_node'
    lane_detect_calibration_file = 'ros_racer_calibration.yaml'
    board_calibration_file = 'adafruit_twist_calibration.yaml'

    lane_detection_config = os.path.join(
        get_package_share_directory(lane_detection2_package),
        'config',
        lane_detect_calibration_file)

    board_config = os.path.join(
        get_package_share_directory(pkg),
        'config',
        board_calibration_file)

    original_topic_name = '/cmd_vel'
    new_topic_name = LaunchConfiguration('topic_name', default=original_topic_name)

    ld = LaunchDescription()
    adafruit_twist_node = Node(
        package=pkg,
        executable=node_name,
        output='screen',
        remappings=[(original_topic_name,new_topic_name)],
        parameters=[lane_detection_config, board_config])
    ld.add_action(adafruit_twist_node)
    return ld
