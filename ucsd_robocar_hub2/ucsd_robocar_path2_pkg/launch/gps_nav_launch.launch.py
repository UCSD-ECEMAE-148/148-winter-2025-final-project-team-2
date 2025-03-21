from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    path2_pkg = 'ucsd_robocar_path2_pkg'
    config_file = 'gps_path_provider.yaml'
    gps_node_name = 'gps_path_provider_node'
    error_node_name = 'gps_error_node'

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(path2_pkg),
        'config',
        config_file)

    path_node = Node(
        package=path2_pkg,
        executable=gps_node_name,
        output='screen',
        parameters=[config])

    error_node = Node(
        package=path2_pkg,
        executable=error_node_name,
        output='screen')

    ld.add_action(path_node)
    ld.add_action(error_node)
    return ld