# ucsd_robocar_actuator2_pkg 

<img src="ucsd_ros2_logos.png">

<div>

## Table of Contents
  - [**Dependencies**](#dependencies)
    - [adafruit_servokit](#adafruit_servokit)
    - [pyVesc](#pyVesc)
  - [**Nodes**](#nodes)
    - [vesc_twist_node](#vesc_twist_node)
    - [adafruit_twist_node](#adafruit_twist_node)
    - [adafruit_servo_node](#adafruit_servo_node)
    - [adafruit_continuous_servo_node](#adafruit_continuous_servo_node)
  - [**Topics**](#topics)
    - [cmd_vel](#topics)
    - [servo](#topics)
    - [continuous_servo](#topics)
  - [**Launch**](#launch)
    - [vesc_twist](#vesc_twist)
    - [adafruit_twist](#adafruit_twist)
    - [adafruit_servo](#adafruit_servo)
    - [adafruit_continuous_servo](#adafruit_continuous_servo)
  - [**Troubleshooting**](#troubleshooting)
    - [Throttle and steering not working](#throttle-and-steering-not-working)

<div align="center">

## Dependencies

</div>

- **Adafruit:** Plenty of information on how to use the adafruit_servokit libraries can be found <a href="https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython" >here</a> and <a href="https://github.com/adafruit/Adafruit_CircuitPython_ServoKit" >here</a> 
- **VESC:** Plenty of information on how to use the vesc python libraries can be found <a href="https://pyvesc.readthedocs.io/en/latest/" >here</a> and <a href="https://github.com/LiamBindle/PyVESC" >here</a> 

<div align="center">

## Nodes

</div>

### **vesc_twist_node**

Associated file: **vesc_twist_node.py**

Associated Topics:
- Subscribes to [cmd_vel](#topics) 

This node subscribes to the [cmd_vel](#topics) topic. This message structure is common throughout many packages in ROS and makes integrating them much easier. In the message, **linear.x** is for the forwards/backwards motion and **angular.z** is for the steering/yaw motion of the robot.
The message must then be decoded to be compatible with the [**pyVesc**](#pyVesc) module.


### **adafruit_twist_node**

Associated file: **adafruit_twist_node.py**

Associated Topics:
- Subscribes to [cmd_vel](#topics) 

This node subscribes to the [cmd_vel](#topics) topic. This message structure is common throughout many packages in ROS and makes integrating them much easier. In the message, **linear.x** is for the forwards/backwards motion and **angular.z** is for the steering/yaw motion of the robot.
The message must then be decoded to be compatible with the [**adafruit_servokit**](#adafruit_servokit) module where the decoded **linear.x** and **angular.z** commands are sent to **channel 2** and **channel 1** respectively.

### **adafruit_servo_node**

Associated file: **adafruit_servo_node.py**

Associated Topics:
- Subscribes to [**servo**](#topics)

This node subscribes to the [**servo**](#topics) topic. Then use the [**adafruit_servokit**](#adafruit_servokit)
module and is left to the user to decide which **channel** to send signals to.

The bus and channel numbers are parameters that can be changed as needed by modifying the **adafruit_servo_calibration.yaml** file in the config directory.

### **adafruit_continuous_servo_node**

Associated file: **adafruit_continuous_servo_node.py**

Associated Topics:
- Subscribes to [**continuous_servo**](#topics)

This node subscribes to the [**continuous_servo**](#topics) topic. Then use the [**adafruit_servokit**](#adafruit_servokit)
module and is left to the user to decide which **channel** to send signals to.

The bus and channel numbers are parameters that can be changed as needed by modifying the **adafruit_continuous_servo_calibration.yaml** file in the config directory.


<div align="center">

## Topics

</div>

| Nodes |  Msg Type | Subscribed Topics | info |
| ------ | ------ | ------ | ------ |
| vesc_twist_node        | geometry_msgs.msg.Twist | /cmd_vel | linear.x (forwards/backwards) angular.z (steering) ranges: [-1,1] |
| adafruit_twist_node    | geometry_msgs.msg.Twist | /cmd_vel | linear.x (forwards/backwards) angular.z (steering) ranges: [-1,1] |
| adafruit_servo_node    | std_msgs.msg.Float32 | /servo    | value range (degrees): [0,180] |
| adafruit_continuous_servo_node | std_msgs.msg.Float32 | /continuous_servo | value range: [-1,1] |

Examples of publishing messages from command line:

- ackerman_drive: `ros2 topic pub  -r 100 /teleop ackermann_msgs/msg/AckermannDriveStamped "drive: {steering_angle: 0.0, speed: 0.0}"`
- cmd_vel: `ros2 topic pub -r 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`
- servo: `ros2 topic pub /servo std_msgs/msg/Float32 "{data: 90.0}"`
- continuous_servo : `ros2 topic pub /continuous_servo std_msgs/msg/Float32 "{data: 0.0}"`


<div align="center">

## Launch

</div>

### **vesc_twist**

Associated file: **vesc_twist.launch.py**

This file launches the [vesc_twist_node](#vesc_twist_node) node.

`ros2 launch ucsd_robocar_actuator2_pkg vesc_twist.launch.py`

### **adafruit_twist**

Associated file: **adafruit_twist.launch.py**

This file launches the [adafruit_twist_node](#adafruit_twist_node) node.

`ros2 launch ucsd_robocar_actuator2_pkg adafruit_twist.launch.py`

### **adafruit_servo**

Associated file: **adafruit_servo.launch.py**

This file launches the [adafruit_servo_node](#adafruit_servo_node) node.

`ros2 launch ucsd_robocar_actuator2_pkg adafruit_servo.launch.py`

### **adafruit_continuous_servo**

Associated file: **adafruit_continuous_servo.launch.py**

This file launches the [adafruit_continuous_servo_node](#adafruit_continuous_servo_node) node.

`ros2 launch ucsd_robocar_actuator2_pkg adafruit_continuous_servo.launch.py`


## **Troubleshooting**

#### **Throttle and steering not working** 

If the throttle and/or steering are unresponsive, then follow the procedure below to potentially resolve the issue.

1. Make sure ESC is turned on
1. Make sure battery is plugged in
1. Make sure battery has a charge
1. -  (for adafruit) Make sure servo and ESC wires are plugged into (for adafruit) the correct channels on the pwm board and that the cables from the adafruit board are connected correctly to the jetson (**BE CAREFUL HERE, POWER DOWN CAR _COMPLETELY_ WHEN RE-WIRING!**)
   -  (for Vesc) Make sure servo and DC motor harnes cables are plugged into vesc and that the usb cable for vesc is plugged into Jetson
1. Check if **host jetson** can recognize the actuators (for adafruit) `sudo i2cdetect -y 1` (for vesc) `ls /dev/ttyACM0`
1. Check that the **docker container** can recognize the actuators (for adafruit) `sudo i2cdetect -y 1` (for vesc) `ls /dev/ttyACM0`
1. Make sure adafruit/Vesc was plugged in all the way into its GPIO/USB socket (**if unplugged, the docker container needs to be restarted in ordered to be recognized**)
1. Check to see if the steering and throttle topics are publishing data `ros2 topic echo /steering` and `ros2 topic echo /throttle`
1. Verify that the throttle values found in [**calibration_node**](#calibration_node) were loaded properly when running [**camera navigation**](#camera_nav_launch_py) (Values will be printed to the terminal first when running the launch file) 
1. Restart ROS2 daemon  `ros2 daemon stop` then `ros2 daemon start`
1. Reboot if none of the above worked and try again `sudo reboot now`

If the Throttle and steering are still not working after trying the procedure above, then it could be a hardware issue. (Did the car crash?)
