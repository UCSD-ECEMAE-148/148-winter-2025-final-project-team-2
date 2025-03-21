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


# cmd=['ros2', 'bag', 'record', '-o', 'sven_cbf2', '/teleop', '/scan', '/imu_topic', '/odom', '/slam_out_pose'],
# ros2 bag record -o sim_gps -s mcap /capra/robot/geo_odometry /capra/robot/odometry

def update_parameters(car_parameter_input_path):
    with open(car_parameter_input_path, "r") as file:
        yaml_inputs = yaml.load(file, Loader=yaml.FullLoader)
    return yaml_inputs


def generate_launch_description():
    some_pkg = 'ucsd_robocar_nav2_pkg'
    some_config = 'rosbag_config.yaml'

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(some_pkg),
        'config',
        some_config)

    rosbag_params = update_parameters(config)
    topics_str = ", ".join(rosbag_params['topics'])
    topics_list = topics_str.split(", ")
    rosbag_cmd=['ros2', 'bag', 'record', '-o', rosbag_params['rosbag_name'], '-s', rosbag_params['rosbag_file_type']]+ topics_list

    rosbag_execute = ExecuteProcess(
        name='execute_ros_bag',
        cmd=rosbag_cmd,
        output='screen',
        shell='True'
        )
    ld.add_action(rosbag_execute)
    return ld
