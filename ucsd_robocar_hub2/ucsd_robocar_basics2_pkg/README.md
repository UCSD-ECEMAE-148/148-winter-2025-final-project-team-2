# ucsd_robocar_basics2_pkg 

<img src="ucsd_ros2_logos.png">

<div>

## Table of Contents
  - [**Config**](#config)
    - [sub_camera_config](#sub_camera_config)
    - [sub_lidar_config](#sub_lidar_config)
    - [subpub_camera_actuator_config](#subpub_camera_actuator_config)
    - [subpub_lidar_actuator_config](#subpub_lidar_actuator_config)
  - [**Nodes**](#nodes)
    - [sub_camera_node](#sub_camera_node)
    - [sub_lidar_node](#sub_lidar_node)
    - [subpub_camera_actuator_node](#subpub_camera_actuator_node)
    - [subpub_lidar_actuator_node](#subpub_lidar_actuator_node)
  - [**Launch**](#launch)
    - [sub_camera_launch](#sub_camera_launch)
    - [sub_lidar_launch](#sub_lidar_launch)
    - [subpub_camera_actuator_launch](#subpub_camera_actuator_launch)
    - [subpub_lidar_actuator_launch](#subpub_lidar_actuator_launch)

<div align="center">

## Config

</div>

### **sub_camera_config**

- Associated packages: **ucsd_robocar_basics2_pkg** (This Package), **ucsd_robocar_sensor2_pkg**
- Associated file: **sub_camera_config.yaml**

This config optionally sets the *live_camera_feed* parameter to visualize the camera feed on the robot
1: camera feed on
0: camera feed off

ex. **sub_camera_config.yaml**
```
sub_camera_node: 
  ros__parameters: 
    live_camera_feed : 1
```

### **sub_lidar_config**

- Associated packages: **ucsd_robocar_basics2_pkg** (This Package)
- Associated file: **sub_lidar_config.yaml**

This config sets the laser mapping of any Lidar (specified in degrees) an optionally setting the *live_laser_feed* parameter which will print the range data from the Lidar
1: lidar feed on
0: lidar feed off

ex. **sub_lidar_config.yaml**
```
sub_lidar_node: 
  ros__parameters: 
    viewing_angle : 360
    front_degree_angle : 0
    right_degree_angle : 90
    left_degree_angle : 270
    live_laser_feed : 1
```

### **subpub_camera_actuator_config**

- Associated packages: **ucsd_robocar_basics2_pkg** (This Package), **ucsd_robocar_sensor2_pkg**
- Associated file: **subpub_camera_actuator_config.yaml**

This config combines the [sub_camera_config](#sub_camera_config) and a new parameter called *Ts* which is the specified controller period (1/frequency)

ex. **subpub_camera_actuator_config.yaml**
```
subpub_camera_actuator_node: 
  ros__parameters: 
    live_camera_feed : 1
    Ts : 0.05
```

### **subpub_lidar_actuator_config**

- Associated packages: **ucsd_robocar_basics2_pkg** (This Package), **ucsd_robocar_sensor2_pkg**
- Associated file: **subpub_lidar_actuator_config.yaml**

This config combines the [sub_lidar_config](#sub_lidar_config) and a new parameter called *Ts* which is the specified controller period (1/frequency)

ex. **subpub_lidar_actuator_config.yaml**
```
subpub_lidar_actuator_node: 
  ros__parameters: 
    viewing_angle : 360
    front_degree_angle : 0
    right_degree_angle : 90
    left_degree_angle : 270
    live_laser_feed : 1
    Ts : 0.05
```

<div align="center">

## Nodes

</div>

### **sub_camera_node**

Associated file: **sub_camera_node.py**

Associated Topics:
- Subscribes to */camera/color/image_raw* topic
- Publishes to *None*

### **sub_lidar_node**

Associated file: **sub_lidar_node.py**

Associated Topics:
- Subscribes to */scan*
- Publishes to *None*

### **subpub_camera_actuator_node**

Associated file: **subpub_camera_actuator_node.py**

Associated Topics:
- Subscribes to */camera/color/image_raw* topic
- Publishes to */drive* topic

### **subpub_lidar_actuator_node**

Associated file: **subpub_lidar_actuator_node.py**

Associated Topics:
- Subscribes to */scan*
- Publishes to */drive* topic

<div align="center">

## Launch

</div>

### **sub_camera_launch**

Associated file: **sub_camera_launch.launch.py**

This file launches the [sub_camera_node](#sub_camera_node) node.

`ros2 launch ucsd_robocar_basics2_pkg sub_camera_launch.launch.py`

### **sub_lidar_launch**

Associated file: **sub_lidar_launch.launch.py**

This file launches the [sub_lidar_node](#sub_lidar_node) node.

`ros2 launch ucsd_robocar_basics2_pkg sub_lidar_launch.launch.py`

### **subpub_camera_actuator_launch**

Associated file: **subpub_camera_actuator_launch.launch.py**

This file launches the [subpub_camera_actuator_node](#subpub_camera_actuator_node) node.

`ros2 launch ucsd_robocar_basics2_pkg subpub_camera_actuator_launch.launch.py`

### **subpub_lidar_actuator_launch**

Associated file: **subpub_lidar_actuator_launch.launch.py**

This file launches the [subpub_lidar_actuator_node](#subpub_lidar_actuator_node) node.

`ros2 launch ucsd_robocar_basics2_pkg subpub_lidar_actuator_launch.launch.py`
