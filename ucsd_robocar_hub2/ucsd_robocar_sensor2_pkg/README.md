# ucsd_robocar_sensor2_pkg 

<img src="ucsd_ros2_logos.png">

<div>

## Table of Contents
  - [**Config**](#config)
    - [ld06](#ld06)
    - [livox_lidar_config](#livox_lidar_config)
    - [rp_lidar](#rp_lidar)
    - [sick_tim_5xx](#sick_tim_5xx)
    - [my_razor](#my_razor)
  - [**Nodes**](#nodes)
    - [webcam_node](#webcam_node)
  - [**Topics**](#topics)
    - [scan](#scan)
    - [camera](#camera)
    - [imu](#imu)
    - [gps](#gps)
  - [**Launch**](#launch)
    - [lidar_bpearl](#lidar_bpearl)
    - [lidar_ld06](#lidar_ld06)
    - [lidar_livox](#lidar_livox)
    - [lidar_rp](#lidar_rp)
    - [lidar_sicktim](#lidar_sicktim)
    - [imu_artemis](#imu_artemis)
    - [camera_intel455](#camera-navigation-calibration-launch)
  - [**Troubleshooting**](#troubleshooting)
    - [Camera not working](#camera-not-working)
    - [Lidar not working](#lidar-not-working)

<div align="center">

## Config

</div>

### **ld06**

Associated file: **ld06.yaml**

Parmater file used in the [lidar_ld06](#lidar_ld06) launch file and is intended to be modified if needed.

### **livox_lidar_config**

Associated file: **livox_lidar_config.json**

Parmater file used in the [lidar_livox](#lidar_livox) launch file and is intended to be modified if needed.

### **rp_lidar**

Associated file: **rp_lidar.yaml**

Parmater file used in the [lidar_rp](#lidar_rp) launch file and is intended to be modified if needed.

### **sick_tim_5xx**

Associated file: **sick_tim_5xx.yaml**

Parmater file used in the [lidar_sicktim](#lidar_sicktim) launch file and is intended to be modified if needed.

### **my_razor**

Associated file: **my_razor.yaml**

Parmater file used in the [imu_artemis](#imu_artemis) launch file and is intended to be modified if needed.


<div align="center">

## Nodes

</div>


### **webcam_node**

Associated file: **webcam_node.py**

Associated Topics:
- Publishes to the [camera](#camera) topic

This node simply reads from the camera with cv2's interface and publishes the image to the
[camera](#camera) topic. Before publishing, the image is reformatted from the cv image format
so it can be passed through the ROS topic message structure.


<div align="center">

## Topics

</div>

### **scan** 
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /scan   | sensor_msgs.msg.LaserScan  | Array containing distance data at specific angles        |


#### **camera**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /camera/color/image_raw | sensor_msgs.msg.Image | Image last read from USB camera image         |

#### **imu**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /imu    |   |sensor_msgs.msg.Imu  | Array containing orientation, angular and linear velocities


#### **gps**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /fix       | sensor_msgs/NavSatFix | Array containing latitude, longitude and altitude          |


<div align="center">

## Launch

</div>

#### **lidar_bpearl**

Associated file: **lidar_bpearl.launch.py**

Launch file that will start the bpearl node provided by manufacturer.

Here is a link to the manufactures official Node: <a href="https://github.com/RoboSense-LiDAR/rslidar_sdk" >bpearl node</a>

#### **lidar_ld06**

Associated file: **lidar_ld06.launch.py**

Launch file that will start the bpearl node provided by manufacturer but with the configuration specified in the [ld06](#ld06) config file.

Here is a link to the manufactures official Node: <a href="https://github.com/linorobot/ldlidar/tree/ros2" >ld06 node</a>

#### **lidar_livox**

Associated file: **lidar_livox.launch.py**

Launch file that will start the bpearl node provided by manufacturer but with the configuration specified in the [livox_lidar_config](#livox_lidar_config) config file.

Here is a link to the manufactures official Node: <a href="https://github.com/Livox-SDK/livox_ros2_driver" >livox node</a>

#### **lidar_rp**

Associated file: **lidar_rp.launch.py**

Launch file that will start the bpearl node provided by manufacturer but with the configuration specified in the [rp_lidar](#rp_lidar) config file.

Here is a link to the manufactures official Node: <a href="https://github.com/youngday/rplidar_ros2" >rp node</a>

#### **lidar_sicktim**

Associated file: **lidar_sicktim.launch.py**

Launch file that will start the bpearl node provided by manufacturer but with the configuration specified in the [sick_tim_5xx](#sick_tim_5xx) config file. 

Here is a link to the manufactures official Node: <a href="https://github.com/SICKAG/sick_scan2" >sicktim node</a>

#### **imu_artemis**

Associated file: **imu_artemis.launch.py**

Launch file that will start the bpearl node provided by manufacturer but with the configuration specified in the [my_razor](#my_razor) config file. 

Here is a link to the manufactures official Node: <a href="https://github.com/ENSTABretagneRobotics/razor_imu_9dof" >imu node</a>

#### **camera_intel455**

Associated file: **camera_intel455.launch.py**

Launch file that will start the bpearl node provided by manufacturer.

Here is a link to the manufactures official Node: <a href="https://github.com/intel/ros2_intel_realsense" >intel455 node</a>

<div align="center">

## Troubleshooting

</div>

#### **Camera not working** 

If while running <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg#camera-navigation-calibration" >camera_nav_calibration_launch.py</a> or <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg#camera-navigation" >camera_nav_launch.py</a> and if the cv2 windows do not open, then follow the procedure below to potentially resolve the issue.

1. Check if host jetson can recognize the camera `lsusb`
1. Check that the docker container can recognize the camera `lsusb`
1. Make sure camera is plugged in all the way into its USB socket (**if camera was unplugged, the docker container needs to be restarted in ordered to be recognized**)
1. verify x-11 forwarding was setup `xclock` should result in an analog clocking popping up
1. See if image feed is coming through in another application like cheese. (Enter `cheese` into terminal window)
1. Check to see if the camera topic is publishing data `ros2 topic echo /camera/color/image_raw`
1. Verify that the calibratrion values found in <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg#camera-navigation-calibration" >camera_nav_calibration_launch.py</a> were loaded properly when running <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg#camera-navigation" >camera_nav_launch.py</a> (Values will be printed to the terminal first when running the camera_nav_launch file) 
1. Restart ROS2 daemon  `ros2 daemon stop` then `ros2 daemon start`
1. Reboot if none of the above worked and try again `sudo reboot now`

If the camera is still not working after trying the procedure above, then it could be a hardware issue. (Did the car crash?)

#### **Lidar not working**

1. Check if host jetson can recognize the lidar (for sick/livox/bpearl) `ping ip_address_of_lidar` (for ld06 and rp) `lsusb`
1. Check that the docker container can recognize the lidar (for sick/livox/bpearl) `ping ip_address_of_lidar` (for ld06 and rp) `lsusb`
1. Make sure lidar is plugged in all the way into its ethernet or USB socket (**if lidar was unplugged, the docker container needs to be restarted in ordered to be recognized**)
1. Check to see if the lidar topic is publishing data `ros2 topic echo /scan`
1. Restart ROS2 daemon  `ros2 daemon stop` then `ros2 daemon start`
1. Reboot if none of the above worked and try again `sudo reboot now`

If the lidar is still not working after trying the procedure above, then it could be a hardware issue. (Did the car crash?)
