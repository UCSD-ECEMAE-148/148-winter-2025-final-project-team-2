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
    pkg_name = 'realsense2_camera'
    launch_file = 'rs_launch.py'

    ld = LaunchDescription()
    intel_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(pkg_name),
                    'launch',
                    launch_file)
            )
        )

    ld.add_action(intel_launch)
    return ld
