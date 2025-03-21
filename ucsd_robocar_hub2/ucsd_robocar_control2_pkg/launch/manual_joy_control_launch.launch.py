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
    some_config = 'manual_joy_control_config.yaml'
    ctrl_pkg ='ucsd_robocar_control2_pkg'
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(ctrl_pkg),
        'config',
        some_config)

    joy_node=Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters = [config])

    joy_teleop_twist_node=Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters = [config])

    ld.add_action(joy_node)
    ld.add_action(joy_teleop_twist_node)
    return ld
