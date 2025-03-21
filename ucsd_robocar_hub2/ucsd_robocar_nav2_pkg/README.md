# ucsd_robocar_nav2_pkg 

<img src="ucsd_ros2_logos.png">

<div>

## Table of Contents
  - [**Config**](#config)
    - [car_config](#car_config)
    - [node_config](#node_config)
    - [pkg_locations_ucsd](#pkg_locations_ucsd)
    - [node_pkg_locations_ucsd](#node_pkg_locations_ucsd)
  - [**Launch**](#launch)
    - [all_components](#all_components)
    - [all_nodes](#all_nodes)

<div align="center">

## Config

</div>

### **car_config**

- Associated package: **ucsd_robocar_nav2_pkg** (This Package)
- Associated file: **car_config.yaml**

This file contains a list of all the components **(sensors/actuators)** available for ucsd_robocar framework and acts as a "switch" (1:true, 0:false) to activate the corresponding nodes in this navigation package.


ex. **car_config.yaml**
```
sick: 0
livox: 0
bpearl: 0
rp_lidar: 0
ld06: 1
webcam: 0
intel: 0
oakd: 0
zed: 0
artemis: 1
ublox: 0
vesc: 1
adafruit: 0
adafruit_servo: 0
adafruit_continuous_servo: 0
esp32: 0
stm32: 0
bldc_sensor: 0
bldc_no_sensor: 0
```

This config corresponds to the following nodes being activated:
- sick
- webcam
- ublox
- vesc
- bldc_sensor

### **node_config**

- Associated package: **ucsd_robocar_nav2_pkg** (This Package)
- Associated file: **node_config.yaml**

Similar to [**car_config**](#car_config), this file contains a list of all the nodes/launch files for various different objectives available for ucsd_robocar framework and acts as a "switch" (1:true, 0:false) to activate the corresponding nodes in this navigation package.

ex. **node_config.yaml**
```
# sensors/hardware
all_components: 1

# camera navigation
camera_nav_calibration: 0
camera_nav: 0

# recording data 
rosbag_launch: 1
ros_bridge_launch: 0

# lidar navigation
lidar_navigation_launch: 0
simple_obstacle_detection_launch: 0

# rviz
sensor_visualization: 0

# control
manual_joy_control_launch: 1
pid_launch: 0
lqr_launch: 0
lqg_launch: 0
mpc_launch: 0

# path planner
path_nav: 0
```

This config corresponds to the following nodes being activated:
- all_components
- rosbag_launch
- manual_joy_control_launch

### **pkg_locations_ucsd**

- Associated package: **ucsd_robocar_nav2_pkg** (This Package)
- Associated file: **pkg_locations_ucsd.yaml**

This file contains a list of all corresponding packages, launch files, sensor type, and desired published topics for all the components (sensors/actuators) available for ucsd_robocar framework. The only parameter that should be changed (if needed) is the ['topics_published'] arguement to suit the needs of the specific problem.

This file is formatted the way it is because it is used to create a dynmically built launch file (more details in the [**all components**](#all_components) launch file).

ex. **pkg_locations_ucsd.yaml**
```
# 
# param: ['package', 'launch_file', 'sensor_type', 'topics_published']
#

# Lidars
sick : ['ucsd_robocar_sensor2_pkg', 'lidar_sicktim.launch.py','lidar', '/scan']
livox : ['ucsd_robocar_sensor2_pkg','lidar_livox.launch.py','lidar', '/scan']
bpearl : ['ucsd_robocar_sensor2_pkg','lidar_bpearl.launch.py','lidar', '/scan']
rp_lidar : ['ucsd_robocar_sensor2_pkg', 'lidar_rp.launch.py','lidar', '/scan']
ld06 : ['ucsd_robocar_sensor2_pkg','lidar_ld06.launch.py','lidar', '/scan']

# IMU
artemis: ['ucsd_robocar_sensor2_pkg', 'imu_artemis.launch.py', 'imu', '/imu']

# GPS
ublox: ['ucsd_robocar_sensor2_pkg', 'gps_ublox.launch.py', 'gps','/gps_topic_name']

# Cameras
webcam: ['ucsd_robocar_sensor2_pkg', 'camera_webcam.launch.py', 'camera', '/camera/color/image_raw']
intel: ['ucsd_robocar_sensor2_pkg', 'camera_intel.launch.py', 'camera', '/camera/color/image_raw']

# Actuator
vesc: ['ucsd_robocar_actuator2_pkg', 'vesc_twist.launch.py','motor', '/cmd_vel']
adafruit: ['ucsd_robocar_actuator2_pkg', 'adafruit_twist.launch.py','motor', '/cmd_vel']
adafruit_servo: ['ucsd_robocar_actuator2_pkg', 'adafruit_servo.launch.py','motor', '/servo']
adafruit_continuous_servo: ['ucsd_robocar_actuator2_pkg', 'adafruit_continuous_servo.launch.py','motor', '/continuous_servo']
```

### **node_pkg_locations_ucsd**

- Associated package: **ucsd_robocar_nav2_pkg** (This Package)
- Associated file: **node_pkg_locations_ucsd.yaml**

Similar to [node_pkg_locations_ucsd](#node_pkg_locations_ucsd), this file contains a list of all corresponding packages and the names of the actual launch files available for ucsd_robocar framework.

This file is formatted the way it is because it is used to create a dynmically built launch file (more details in the [**all_nodes**](#all_nodes) launch file).

ex. **node_pkg_locations_ucsd.yaml**
```
# 
# param: ['package', 'launch_file']
#
# sensors/hardware
all_components: ['ucsd_robocar_nav2_pkg', 'all_components.launch.py']

# camera navigation
camera_nav_calibration: ['ucsd_robocar_lane_detection2_pkg', 'camera_nav_calibration.launch.py']
camera_nav: ['ucsd_robocar_lane_detection2_pkg', 'camera_nav.launch.py']

# recording/bridging data 
rosbag_launch: ['ucsd_robocar_nav2_pkg', 'rosbag_launch.launch.py']
ros_bridge_launch: ['ucsd_robocar_nav2_pkg', 'ros_bridge_launch.launch.py']

# obstacle detection
simple_obstacle_detection_launch: ['ucsd_robocar_path2_pkg', 'simple_obstacle_detection_launch.launch.py']

# rviz
sensor_visualization: ['ucsd_robocar_nav2_pkg', 'sensor_visualization.launch.py']

# control
manual_joy_control_launch: ['ucsd_robocar_control2_pkg', 'manual_joy_control_launch.launch.py']
pid_launch: ['ucsd_robocar_control2_pkg', 'pid_launch.launch.py']
lqr_launch: ['ucsd_robocar_control2_pkg', 'lqr_launch.launch.py']
lqg_launch: ['ucsd_robocar_control2_pkg', 'lqg_launch.launch.py']
mpc_launch: ['ucsd_robocar_control2_pkg', 'mpc_launch.launch.py']


# path planner
path_nav: ['ucsd_robocar_path2_pkg', 'path_launch.launch.py']
```

<div align="center">

## Launch

</div>

#### **all_components**

- Associated package: **ucsd_robocar_nav2_pkg**
- Associated file: **all_components.launch.py**

This file will launch all the sensors and actuators specified as **active** in [**car_config**](#car_config)

`ros2 launch ucsd_robocar_nav2_pkg all_components.launch.py`

#### **all_nodes**

- Associated package: **ucsd_robocar_nav2_pkg**
- Associated file: **all_nodes.launch.py**

This file will launch all the nodes/launch files specified as **active** in [**node_config**](#node_config)

`ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py`
