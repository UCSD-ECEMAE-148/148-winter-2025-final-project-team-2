from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'ucsd_robocar_nav2_pkg'
    urdf_file = 'f1_10_robot.urdf'
    desc_dir = get_package_share_directory(pkg_name)

    urdf_file_path = os.path.join(
        desc_dir,
        'urdf',
        urdf_file
    )

    with open(urdf_file_path, 'r') as infile:
        urdf_content = infile.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[
            {'robot_description':urdf_content,
            'use_sim_time': False}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)

    return ld
