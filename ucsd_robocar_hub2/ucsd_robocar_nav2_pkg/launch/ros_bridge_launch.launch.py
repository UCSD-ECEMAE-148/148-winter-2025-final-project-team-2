import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml


def generate_launch_description():
    ld = LaunchDescription()
    ros_bridge_execute = ExecuteProcess(
        name='execute_ros_bridge',
        cmd=['ros2', 'run', 'ros1_bridge', 'dynamic_bridge','--bridge-all-topics'],
        output='screen',
        shell='True'
        )
    ld.add_action(ros_bridge_execute)
    return ld
