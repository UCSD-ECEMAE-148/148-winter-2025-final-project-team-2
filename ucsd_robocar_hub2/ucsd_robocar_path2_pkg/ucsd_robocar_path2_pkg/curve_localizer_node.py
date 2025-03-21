from cmath import pi
from turtle import right
import numpy as np
from control.matlab import *
import time
import csv

from .path_submodule.CurveLocalizer import CurveLocalizer
import matplotlib.pyplot as plt

#ROS2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu # Might not be needed depending on the output of the gap detector
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped


# CurveLocalizer Class: 
# By taking in the raw laser scan, we compute the upcoming curvature in a look ahead distance away
# (30 degrees either direction from zero), 

NODE_NAME = 'curve_localizer_node'
MODE = 1 # 1: pure curve detection, #2: everything else
LIDAR_TYPE = 1 # 1: LD06, 2: Hokuyo

LIDAR_TOPIC_NAME = '/scan'
ODOM_TOPIC_NAME = '/odom'
STEERING_TOPIC_NAME = '/teleop'

OUTPUT_TOPIC_NAME = '/path_curvature'
DEBUG_CURVE_TOPIC_NAME = '/Shrey_fucked_up'
class CurveLocalizerNode(Node):
    def __init__(self):
        # ROS2 communication 
        super().__init__(NODE_NAME)
        self.publiser_ = self.create_publisher(Float32MultiArray, OUTPUT_TOPIC_NAME, 10)
        self.curve_publisher = self.create_publisher(Float32MultiArray, DEBUG_CURVE_TOPIC_NAME, 10)
        self.lidar_sub = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.laserScanCB, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odomCB, 10)
        self.heading_sub = self.create_subscription(AckermannDriveStamped, STEERING_TOPIC_NAME, self.steeringCB, 10)

        # Initialize default parameters
        map_Dir_default = [0,0,0,0,0,0,0]
        map_Dir_default = list(np.array(map_Dir_default, dtype = 'float'))
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mode', 2),
                ('mapDir', map_Dir_default),
                ('sampleDist', 0.5),
                ('startIdx', 0),
                ('history_size', 3),
                ('laserFOV', 60),
                ('mapWindow', 7)
            ])
        MODE = self.get_parameter('mode').value
        mapDir = self.get_parameter('mapDir').value
        sampleDist = self.get_parameter('sampleDist').value
        startIdx = self.get_parameter('startIdx').value
        history_size = self.get_parameter('history_size').value
        laserFOV = self.get_parameter('laserFOV').value
        mapWindow = self.get_parameter('mapWindow').value

        # Laser Scan Parameters
        self.laserFOV = laserFOV # FOV in degrees, must be even
        self.newLidarAvailable = False

        # Map Parameters
        self.mapWindow = mapWindow

        # Odometry Data
        self.vx = 0
        self.steering = 0
        self.newVelAvailable = True
        self.newSteerAvailable = True

        self.time_now = time.time()
        self.plotting_curvature = []
        self.cl = CurveLocalizer(mapDir, sampleDist, history_size, startIdx, mapWindow, self.time_now)

        if MODE == 2:
            self.Ts = 1/100
            self.create_timer(self.Ts, self.run)
        
    # Callback Functions
    def laserScanCB(self, msg):
        dtheta = msg.angle_increment # Given in radians
        data = msg.ranges
        angles = list(np.linspace(msg.angle_min, msg.angle_max, int((msg.angle_max-msg.angle_min)/dtheta)+1))

        # Get Desired Scan Areas
        fov = self.laserFOV*pi/180 #convert fov to radians
        idx_count = int(fov/(dtheta)) #number of indicies to consider on either side

        if LIDAR_TYPE == 1: # LD06
            right90Idx = int((pi/2)/dtheta)
            left90Idx = int((3*pi/2)/dtheta)
            lefttheta = angles[left90Idx:left90Idx+idx_count]
            leftr = data[left90Idx:left90Idx+idx_count]

            righttheta = angles[right90Idx-idx_count:right90Idx]
            rightr = data[left90Idx:left90Idx+idx_count]
        
        if LIDAR_TYPE == 2: # Hokuyo
            right90Idx = int((pi/4)/dtheta)
            left90Idx = int(right90Idx+pi/dtheta)
            lefttheta = angles[left90Idx-idx_count:left90Idx]
            leftr = data[left90Idx-idx_count:left90Idx]       
            
            righttheta = xangles[right90Idx:right90Idx+idx_count]
            rightr = data[right90Idx:right90Idx+idx_count]

         # Fiilter for NaN and Inf points

        temp = np.vstack((lefttheta, leftr))
        temp = temp[:,~np.isnan(temp).any(axis=0)]
        temp = temp[:,~np.isinf(temp).any(axis=0)]
        lefttheta = temp[0,:]
        leftr = temp[1,:]

        temp = np.vstack((righttheta, rightr))
        temp = temp[:,~np.isnan(temp).any(axis=0)]
        temp = temp[:,~np.isinf(temp).any(axis=0)]
        righttheta = temp[0,:]
        rightr = temp[1,:]


        leftx,lefty = self.polar2cart(leftr,lefttheta)
        rightx,righty = self.polar2cart(rightr,righttheta)
        
        if (len(leftx) == 0 and len(rightx) == 0) or (len(lefty) == 0 and len(righty) == 0):
            print(f"WARNING: Both Left and Right coordinates are NaN. Ignoring the scan ...")
            return

        print('left: ', len(leftx), 'right: ', len(rightx))
        left_cent, left_curv = self.cl.fitcircle(leftx, lefty)
        right_cent, right_curv = self.cl.fitcircle(rightx, righty)

        if MODE == 1:
            print('left_curv: ', left_curv, 'right_curv: ', right_curv)
            # Used for plotting the curvature
            self.cl.addMeasurement((right_curv+left_curv)/2) # change this in the future (choose curv)
            self.plotting_curvature.append((right_curv+left_curv)/2)

            ########### CSV FILE FOR CURVATURE ############
            # with open('/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_path2_pkg/ucsd_robocar_path2_pkg/plotting_curv.csv', 'w') as outfile:
            #     writer = csv.writer(outfile)
            #     writer.writerow(self.plotting_curvature)

        else:
            self.cl.addMeasurement((right_curv+left_curv)/2) # change this in the future (choose curv)
            self.plotting_curvature.append((right_curv+left_curv)/2)
            # with open('/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_path2_pkg/ucsd_robocar_path2_pkg/plotting_curv.csv', 'w') as outfile:
            #     writer = csv.writer(outfile)
            #     writer.writerow(self.plotting_curvature)
            self.newLidarAvailable = True

        msg = Float32MultiArray()
        msg.data = [float(left_curv), float(right_curv)]
        self.curve_publisher.publish(msg)

    def odomCB(self, msg):
        self.vx = msg.twist.twist.linear.x
        self.newVelAvailable = True

    def steeringCB(self, msg):
        self.steering = msg.drive.steering_angle
        self.newSteerAvailable = True

    def polar2cart(self, data, angles):
        # remove nan values
        nan_check = np.vstack((data,angles))
        nan_check = nan_check[:,~np.isnan(nan_check).any(axis=0)]

        data = nan_check[0,:]
        angles = nan_check[1,:]

        x = np.multiply(data,np.cos(angles))
        y = np.multiply(data,np.sin(angles))

        return x,y

    def run(self):
        if self.newLidarAvailable and self.newVelAvailable and self.newSteerAvailable:
            idx = self.cl.computePosition(self.vx, self.steering, time.time(), True)
            msg = Float32MultiArray()
            msg.data = [self.cl.map[idx], 0.5]
            self.publiser_.publish(msg)
        if ~self.newLidarAvailable and self.newVelAvailable and self.newSteerAvailable:
            idx = self.cl.computePosition(self.vx, self.steering, time.time(), False)
            msg = Float32MultiArray()
            msg.data = [self.cl.map[idx], 0.5]
            self.publiser_.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)
    curvloc_pub = CurveLocalizerNode()
    try:
        rclpy.spin(curvloc_pub)
        curvloc_pub.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        curvloc_pub.get_logger().info(f'{NODE_NAME} shut down successfully')

if __name__ == '__main__':
    main()
