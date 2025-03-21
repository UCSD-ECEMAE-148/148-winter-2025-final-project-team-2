import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace



import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ucsd_robocar_sensor2_pkg',  # Replace with your actual package name
            executable='ublox_gps_node',  # Replace with your node script name >
            name='ublox_gps_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 38400},
            ],
        )
    ])
