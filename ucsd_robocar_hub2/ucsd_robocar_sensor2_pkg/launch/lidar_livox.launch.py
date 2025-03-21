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
    some_package = 'livox_ros2_driver'
    some_launch = 'livox_lidar_rviz_launch.py'

    ld = LaunchDescription()
    counter_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(some_package),
                    some_launch)
            )
        )
    ld.add_action(counter_launch)
    return ld
