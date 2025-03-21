# ucsd_robocar_lane_detection2_pkg 

<img src="ucsd_ros2_logos.png">

<div>

## Table of Contents
  - [**Work Flow To Use This Repository**](#work-flow-to-use-this-repository)
  - [**Config**](#config)
    - [ros_racer_calibration](#ros_racer_calibration)
  - [**Nodes**](#nodes)
    - [lane_detection_node](#lane_detection_node)
    - [lane_guidance_node](#lane_guidance_node)
    - [calibration_node](#calibration_node)
  - [**Topics**](#topics)
  - [**Launch**](#launch)
    - [camera navigation calibration](#camera-navigation-calibration-launch)
    - [camera navigation](#camera_nav_launch_py)
  - [**Tools**](#tools)
    - [ROS1&2 Guide Books](#ros12-guide-books)
    - [Calibration Tutorial](#calibration-tutorial)
  - [**Troubleshooting**](#troubleshooting)
    - [Camera not working](#camera-not-working)
    - [Throttle and steering not working](#throttle-and-steering-not-working)
  - [**Demonstration videos**](#demonstration-videos)
    - [Lane detection example with yellow filter](#lane-detection-example-with-yellow-filter)
    - [Blue color detection](#blue-color-detection)
    - [Yellow color detection and line width specification](#yellow-color-detection-and-line-width-specification)
    - [Throttle and steering](#throttle-and-steering)
    - [Manipulating image dimensions](#manipulating-image-dimensions)

<div align="center">

## **Work Flow To Use This Repository**

</div>

1. Pull docker image <a href="https://hub.docker.com/r/djnighti/ucsd_robocar" >**here**</a> for this repo. The docker image contains all dependecies for plug-n-play use and also provides neccessary instructions for how to run the docker container.
2. Make sure that the <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg">**car_config in the ucsd_robocar_nav2_pkg**</a> has been updated to reflect your specific cars configuration of sensors and actuators. Once this updated, make sure to recompile the packages in the ros2_ws.
3. Calibrate the camera, throttle and steering values using the [**calibration_node**](#calibration_node)
4. Launch [**camera navigation**](#camera_nav_launch_py)
5. Tune parameters in the calibration until desired behavior is achieved

<div align="center">

## Config

</div>

### **ros_racer_calibration**

Associated file: **ros_racer_calibration.yaml**


<div align="center">

## Nodes

</div>


### **lane_detection_node**

Associated file: **lane_detection.py**

Associated Topics:
- Subscribes to the **camera** [**Topics**](#topics)
- Publishes to the **centroid** [**Topics**](#topics)

This node subscribes from the [**camera**](#camera) topic and uses opencv to identify line
information from the image, and publish the information of the lines centroid to the [**centroid**](#centroid). 

The color scheme is defined as follows:

- 2 contours : green bounding boxes and a blue average centroid
- 1 contour : green bounding box with a single red centroid

Below show the image post processing techniques, cv2 methods and the logic applied respectively.

<div align="center">
  <img src="filtering_process.png">
  <img src="applying_methods.png">
  <img src="applying_logic.png">
</div>

### **lane_guidance_node**

Associated file: **lane_guidance.py**

Associated Topics:
- Subscribes to the **centroid** [**Topics**](#topics)
- Publishes to the **cmd_vel** [**Topics**](#topics)

This node subscribes to the centroid topic, calculates the throttle and steering
based on the centroid value, and then publish them to their corresponding topics.


Throttle is determined by a throttle scheduler that uses the following 4 parameters:
- error between road mark to be tracked (ie. the **centroid** which is found in the [**lane_detection_node**](#lane_detection_node)) and the heading of the car.
- error threshold (determined in the [**calibration_node**](#calibration_node))
- max throttle (determined in the [**calibration_node**](#calibration_node))
- min throttle (determined in the [**calibration_node**](#calibration_node))
depending on the value of the error, the car will change its throttle value linearly.

| Error Threshold State| Throttle Response |
| ------ | ------ |
| error < error_threshold | car throttle = **max throttle** |
| error > error_threshold | min throttle <= car throttle = **variable throttle** < max throttle |

Steering is based on a PID controller that uses the following 5 parmeters
- error between road mark to be tracked (ie. the **centroid** which is found in the [**lane_detection_node**](#lane_detection_node)) and the heading of the car.
- error threshold (determined in the [**calibration_node**](#calibration_node))
- PID constants (determined in the [**calibration_node**](#calibration_node))

| Error Threshold State| Steering Response |
| ------ | ------ |
| error < error_threshold | car steers **straight** |
| error > error_threshold | car steers **toward error** |

### **calibration_node**

Associated file: **calibration_node.py**

Associated Topics:
- Subscribes to the **centroid** [**Topics**](#topics)
- Publishes to the **cmd_vel** [**Topics**](#topics)


**These values are saved automatically to a configuration file, so just press** `control-c` **when the car is calibrated.**

Calibrate the camera, throttle and steering in this node by using the sliders to find:
- the right color filter 
- desired image dimmensions
- throttle values for both the optimal condtion (error = 0) and the non optimal condtion (error !=0) AKA go max throttle when error=0 and go slower if error !=0
- PID values to adjust the steering response. Values too high or low can make the car go unstable (oscillations in the cars trajectory)
- Steering values to calibrate max left/right and forward directions

| Property   | Info |
| ----------  | --------------------- |
| lowH, highH | Setting low and high values for Hue  | 
| lowS, highS | Setting low and high values for Saturation | 
| lowV, highV | Setting low and high values for Value | 
| gray_lower | Specify lower gray value threshold | 
| Inverted_filter | Specify to create an inverted color tracker | 
| kernal_size | Specify size of the kernal to be convolved with the image for erosion/dilation transformations | 
| erosion_itterations | Specify the number of times to perform an erosion transfrom (more itterations ~ "shrink" white pixels) | 
| dilation_itterations | Specify the number of times to perform an dilation transfrom (more itterations ~ "grow" white pixels)  | 
| min_width, max_width | Specify the width range of the line to be detected  | 
| number_of_lines | Specify the number of lines to be detected  | 
| error_threshold | Specify the acceptable error the robot will consider as approximately "no error" | 
| camera_centerline | Specify the center of the camera frame with respect to the robots central x-axis | 
| frame_width | Specify the width of image frame (horizontal cropping) | 
| rows_to_watch | Specify the number of rows (in pixels) to watch (vertical cropping) | 
| rows_offset | Specify the offset of the rows to watch (vertical pan) | 
| Kp_steering | Specify the proportional gain of the steering |
| Ki_steering | Specify the integral gain of the steering |
| Kd_steering | Specify the derivative gain of the steering |
| Steering_mode | Toggle this slider to switch steering calibration to the following 3 modes. |
| Steering_mode 0 | Left_steering_mode (find max left steering angle) 
| Steering_mode 1 | Straight_steering_mode (find straight steering angle)
| Steering_mode 2 | Right_steering_mode (find max right steering angle)|  
| Steering_value | Specify the steering value to be set in the current steering mode | 
| Throttle_mode | Toggle this slider to switch throttle calibration to the following 3 modes. |
| Throttle_mode 0 | zero_throttle_mode (find value where car does not move) 
| Throttle_mode 1 | max_throttle_mode (find value for car to move when there is **no error** in steering)
| Throttle_mode 2 | min_throttle_mode(find value for car to move when there is **some error** in steering)| 
| Throttle_value | Specify the throttle value to be set in the current throttle mode | 
| Max_RPM_value | Specify the max possible RPM (if using a VESC) which will be scaled down by throttle mode values | 
| Steering_polarity | Flip left/right direction |
| Throttle_polarity | Flip forwards/backwards direction |
| Test_motor_control | Test PID and throttle behavior chosen from the sliders above in this semi-autonomous mode for tracking road marks |

More morphological transfromations and examples can be found <a href="https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html" >here</a> and <a href="https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html" >here</a>

<div align="center">

## Topics

</div>

| Nodes | Subscribed Topics | Published Topics |
| ------ | ------ | ------ |
| lane_detection_node | /camera/color/image_raw | /centroid |
| lane_guidance_node  | /centroid               | /cmd_vel |
| calibration_node    | /camera/color/image_raw | /cmd_vel  |

<div align="center">

## Launch

</div>

#### **camera navigation calibration**

- Associated package: [**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg)
- Associated file: **camera_nav_calibration.launch.py**
- Associated nodes: [**calibration_node**](#calibration_node)

This file will launch the [**calibration_node**](#calibration_node)

`ros2 launch ucsd_robocar_lane_detection2_pkg camera_nav_calibration.launch.py`

#### **camera navigation**

- Associated package: [**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg)
- Associated file: **camera_nav.launch.py**
- Associated nodes: [**lane_detection_node**](#lane_detection_node) and [**lane_guidance_node**](#lane_guidance_node)

This file will launch both the [**lane_detection_node**](#lane_detection_node) and [**lane_guidance_node**](#lane_guidance_node) and load the parameters determined using the [**calibration_node**](#calibration_node) from

**Before launching, please calibrate the robot first while on the stand!**

`ros2 launch ucsd_robocar_lane_detection2_pkg camera_nav.launch.py`

<div align="center">

## Tools 

</div>

#### ROS1&2 Guide Books

For help with using ROS in the terminal and in console scripts, check out these google doc below to see tables of ROS commands and plenty of examples of using ROS in console scripts.

- <a href="https://docs.google.com/document/d/1DJgVLnu_vN-IXKD3QrQVF3W-JC6RiQPVugHeFAioB58/edit?usp=sharing" >ROS2 Guide Book</a>

#### Calibration Tutorial

Check out this google doc <a href="https://docs.google.com/document/d/1YS5YGbo8evIo9Mlb0J-w2r3bZfju37Zl4UmdaN2CD2A/edit#heading=h.rxwmnnbbd9xn" >here</a> for a tutorial that will walk through the calibration process step by step and explain more about the sliders and their effects.


<div align="center">

## Troubleshooting

</div>

#### **Camera not working** 

See troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_sensor2_pkg#troubleshooting" >here</a>

#### **Throttle and steering not working** 

See troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg#troubleshooting" >here</a>

<div align="center">

## **Demonstration videos** 

</div>

<div align="center">

#### Lane detection example with yellow filter

[![lane detection example with yellow filter](https://j.gifs.com/6WRqXN.gif)](https://youtu.be/f4VrncQ7HU)

</div>

<div align="center">

#### Number of lines to detect
[![Number of lines to detect](https://j.gifs.com/qQ7Lvk.gif)](https://youtu.be/5AVL68BTD0U)

</div>

<div align="center">

#### Error threshold
[![Error threshold](https://j.gifs.com/k28Xmv.gif)](https://youtu.be/ied1TDvpDK4)

</div>

<div align="center">

#### Blue color detection

[![Blue color detection](https://j.gifs.com/PjZoj6.gif)](https://youtu.be/c9rkRHGGea0)

</div>

<div align="center">

#### Yellow color detection and line width specification

[![Yellow color detection and line width specification](https://j.gifs.com/BrLJro.gif)](https://youtu.be/l2Ngtd461DY)

</div>

<div align="center">

#### Throttle and steering

[![Throttle and steering](https://j.gifs.com/362n6p.gif)](https://youtu.be/lhYzs5v6jtI)

</div>

<div align="center">

#### Manipulating image dimensions

[![Manipulating image dimensions](https://j.gifs.com/lR5oR1.gif)](https://youtu.be/_DhG6dduYPs)

</div>

