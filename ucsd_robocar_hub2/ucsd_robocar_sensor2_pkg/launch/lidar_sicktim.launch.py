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
    sensor_pkg = 'ucsd_robocar_sensor2_pkg'
    some_package = 'sick_scan2'
    some_node = 'sick_generic_caller'
    some_config = 'sick_tim_5xx.yaml'

    original_topic_name = 'scan'
    new_topic_name = LaunchConfiguration('topic_name', default=original_topic_name)

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(sensor_pkg),
        'config',
        some_config
        )

    node=Node(
        package=some_package,
        node_executable=some_node,
        output='screen',
        parameters = [config],
        remappings=[(original_topic_name,new_topic_name)]
    )
    ld.add_action(node)
    return ld
