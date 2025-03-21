import pandas as pd
import numpy as np
import math
import pymap3d
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import NavSatFix

NODE_NAME = 'gps_error_node'
ERROR_TOPIC_NAME = '/error'
PATH_TOPIC_NAME = '/gps_trajectory'
POSE_TOPIC_NAME = '/fix'
np.set_printoptions(precision=20)


class GpsError(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.timer = self.create_timer(1.0, self.log_pose)
        self.R = 6378.137E3 # Radius of earth in M
        self.e_cg_y_mag_prev = 0
        self.e_cg_y_mag_curr = 0 
        self.e_cg_y_sign_prev = 1
        # future waypoint
        self.reached_P2 = False
        # initial waypoint
        self.reached_Pi = False
        # final waypoint
        self.reached_PN = False
        self.waypoint_index_P1=0
        self.waypoint_index_P2=self.waypoint_index_P1+1
        # paths complete
        self.path_complete_p12=0.0
        self.path_complete = 0.0

        self.diff_tol = 1.0E-14

        self.P1=[0.0, 0.0]
        self.P2=[0.0, 0.0]

        # threads
        self.pose_thread = MutuallyExclusiveCallbackGroup()
        self.path_thread = MutuallyExclusiveCallbackGroup()
      
        # common class variables
        self.QUEUE_SIZE = 10

        # ROS parameters
        self.declare_parameters(
          namespace='',
          parameters=[
            ('timer_period', 0.05),
            ('distance_to_goal_tolerance', 1.0),
            ('waypoint_look_ahead', 1),
            ('reset_path', 0),
            ('show_logger', 1)
          ])
      
        self.timer_period = self.get_parameter('timer_period').value 
        self.distance_to_goal_tolerance = self.get_parameter('distance_to_goal_tolerance').value 
        self.waypoint_look_ahead = self.get_parameter('waypoint_look_ahead').value
        self.reset_path = self.get_parameter('reset_path').value 
        self.show_logger = self.get_parameter('show_logger').value 

        # error publisher
        self.error_publisher = self.create_publisher(Float32MultiArray, ERROR_TOPIC_NAME, self.QUEUE_SIZE)
        self.error_msg = Float32MultiArray()
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim[0].label = "longitdudinal_error"
        self.error_msg.layout.dim[1].label = "lateral_error"
        self.error_msg.layout.dim[2].label = "heading_error"
        self.error_msg.layout.dim[3].label = "path_complete_%"
        self.error_msg.layout.dim[0].size = 1
        self.error_msg.layout.dim[1].size = 1
        self.error_msg.layout.dim[2].size = 1
        self.error_msg.layout.dim[3].size = 1

        # path subscriber
        self.path_subscriber = self.create_subscription(
            Path, 
            PATH_TOPIC_NAME, 
            self.set_path, 
            qos_profile_sensor_data, 
            callback_group=self.path_thread)
        self.path_subscriber

        # pose subscriber
        self.pose_subscriber = self.create_subscription(
            NavSatFix, 
            POSE_TOPIC_NAME, 
            self.set_pose, 
            self.QUEUE_SIZE, 
            callback_group=self.pose_thread)
        self.pose_subscriber

        # path and pose variables (x:lon, y:lat)
        self.x_pose=0.0
        self.y_pose=0.0
        self.theta_pose=0.0
        self.x_pose_prev=0.0
        self.y_pose_prev=0.0
        self.theta_pose_prev=0.0
        self.theta_robot=0.0
        self.distance_to_previous_pose=0.0
        self.x_path=np.empty((0, 0))
        self.y_path=np.empty((0, 0))
        self.theta_path=np.empty((0, 0))
        self.path_array = np.empty((0, 3))
        self.number_of_waypoints = 0
        self.path_received=False
        self.initial_pose_set=False

        # publish results timer
        self.timer = self.create_timer(self.timer_period, self.publish_error)

    def set_path(self, path_data):
        '''
        x_path: lon
        y_path: lat
        '''
        if not self.path_received:
            self.get_logger().info("Updating PATH")
            x_path=[]
            y_path=[]
            theta_path=[]
            for pose in path_data.poses:
                x_path.append(pose.pose.position.x)
                y_path.append(pose.pose.position.y)
                theta_path.append((pose.pose.orientation.w - math.pi/2 + 4*math.pi) % (2*math.pi))
            
            # create arrays for lat,lon,heading
            self.x_path = np.array(x_path)
            self.y_path = np.array(y_path)
            self.theta_path = np.array(theta_path)
            self.number_of_waypoints = self.x_path.shape[0]

            # update path received flag to start publishing
            self.path_received=True
            
            # display waypoints to track
            self.path_array = np.hstack(
               (np.array(self.x_path).reshape(-1,1), 
                np.array(self.y_path).reshape(-1,1), 
                np.array(self.theta_path).reshape(-1,1)))
            
            self.get_logger().info(
                f'\n path_array: {str(self.path_array)}'
                f'\n number_of_waypoints: {self.number_of_waypoints}'
                )
    
    def set_pose(self, pose_data):
        if not self.initial_pose_set:
            self.x_pose_prev=pose_data.longitude
            self.y_pose_prev=pose_data.latitude
            self.theta_pose_prev=0.0
            self.initial_pose_set=True
        else:
            error_data_check = np.array([pose_data.position_covariance[0], pose_data.position_covariance[4], pose_data.position_covariance[8]])
            if not (np.isnan(error_data_check).any()):
                self.x_pose=pose_data.longitude
                self.y_pose=pose_data.latitude
                self.theta_pose=0.0
            
    def calculate_haversine_distance(self, point1, point2):
        '''
        point1: [lon1, lat1]
        point2: [lon2, lat2]
        '''
        lon1 = point1[0]
        lat1 = point1[1]
        
        lon2 = point2[0]
        lat2 = point2[1]

        dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
        dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = self.R * c
        return d
    
    def calculate_haversine_bearing(self, point1, point2):

        '''
        point1: [lon1, lat1]
        point2: [lon2, lat2]
        '''
        lon1 = point1[0]
        lat1 = point1[1]
        
        lon2 = point2[0]
        lat2 = point2[1]

        # Convert coordinates to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Calculate differences in longitude and latitude
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        # Calculate bearing
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        theta = math.atan2(y, x)
        theta = math.radians(theta*(180/math.pi + 360) % 360)
        return theta

    def calculate_2_closest_points(self):
        # calculate haversine from current pose to every point in list
        error_mag = []
        for waypoint in range(0, self.number_of_waypoints):
            an_error = self.calculate_haversine_distance(
                [self.x_pose, self.y_pose], 
                [self.x_path[waypoint], self.y_path[waypoint]])
            error_mag.append(an_error)

        # find 2 smallest values
        error_mag1, error_mag2 = np.partition(error_mag, 1)[0:2]
        
        # find index in path where the 2 closest points are
        error_mag1_index = np.where(error_mag == error_mag1)[0][0]
        error_mag2_index = np.where(error_mag == error_mag2)[0][0]

        Px1 = self.x_path[error_mag1_index]
        Px2 = self.x_path[error_mag2_index]
        Py1 = self.y_path[error_mag1_index]
        Py2 = self.y_path[error_mag2_index]

        P1 = [Px1, Py1]
        P2 = [Px2, Py2]
        return P1, P2, error_mag1_index
    
    def calculate_slope_between_2_points(self, point1, point2):
        '''
        point1: [lon1, lat1]
        point2: [lon2, lat2]
        '''
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        m = delta_y / delta_x
        return m
    
    def calculate_error_perp_path(self):
        '''
        # x-lon
        # y-lat
        '''   
        # If P2 has been reached, update the waypoint index to track the next points
        # ie. 
        #   @step=0 (P1,P2) -> ([self.x_path[0],self.y_path[0]],[self.x_path[1],self.y_path[1]])
        #   @step=1 (P1,P2) -> ([self.x_path[1],self.y_path[1]],[self.x_path[2],self.y_path[2]])
        #   @step=N-1 (P1,P2) -> ([self.x_path[N-1],self.y_path[N-1]],[self.x_path[N],self.y_path[N]])
        #   @step=N final waypoint reached.
        if self.reset_path:
            self.waypoint_index_P1=0
            self.waypoint_index_P2=1
            self.reached_PN=False
            self.reached_Pi=False
        
        if self.reached_P2 and not self.reached_PN and (self.waypoint_index_P1+1) != (self.number_of_waypoints-1):
            self.waypoint_index_P1+=1
            self.waypoint_index_P2 = self.waypoint_index_P1+1
        
        # P: [lon,lat]
        self.P1=[self.x_path[self.waypoint_index_P1], self.y_path[self.waypoint_index_P1]]
        self.P2=[self.x_path[self.waypoint_index_P2], self.y_path[self.waypoint_index_P2]]
        
        delta_x = self.P2[0] - self.P1[0]
        delta_y = self.P2[1] - self.P1[1]
        if delta_x==0.0:
            delta_x=self.diff_tol

        mp = delta_y / delta_x
        bp = self.P1[1] - self.P1[0]*mp
        mq = -1/mp
        bq = self.y_pose - self.x_pose * mq
        x_cte = (bp-bq) / (mq-mp)
        y_cte = mq*x_cte + bq

        R_x = self.x_pose - self.P1[0]
        R_y = self.y_pose - self.P1[1]
        r_2 = np.power(delta_x, 2) + np.power(delta_y, 2)
        e_cg_y_cross = (R_y * delta_x - R_x * delta_y) / r_2
        
        # path heading
        theta_path_calc = self.calculate_haversine_bearing(
            self.P1, 
            self.P2)

        # lateral error to tracking waypoint
        e_cg_y_mag = self.calculate_haversine_distance(
            [self.x_pose, self.y_pose], 
            [x_cte, y_cte])
        
        # distance from P1 to CTE
        dist_p1_cte = self.calculate_haversine_distance(
            [x_cte, y_cte],
            self.P1)
        
        # distance from P2 to CTE (longitude error from tracking waypoint to the actual waypoint to follow)
        dist_p2_cte = self.calculate_haversine_distance(
            [x_cte, y_cte],
            self.P2)
        
        # distance between two closest points
        dist_p1_p2 = self.calculate_haversine_distance(
            self.P1,
            self.P2)

        # calculate distance to last point in path
        distance_to_goal = self.calculate_haversine_distance(
            [self.x_pose, self.y_pose], 
            [self.x_path[-1], self.y_path[-1]])
        
        # check if reached P2
        if dist_p2_cte <= self.distance_to_goal_tolerance:
            self.reached_P2 = True
        else:
            self.reached_P2 = False

        # calculate distance to first point in path
        if not self.reached_Pi:
            self.path_complete_p12 = 0.0

            # calculate distance to first point
            dist_pi_cte = self.calculate_haversine_distance(
                [x_cte, y_cte], 
                [self.x_path[0], self.y_path[0]])
            
            # check if reached first waypoint
            if dist_pi_cte <= self.distance_to_goal_tolerance:
                self.reached_Pi = True
            else:
                self.reached_Pi = False
        # if robot is begining path, calculate % complete from P1 to P2
        else:
            self.path_complete_p12 = float((dist_p1_cte)/dist_p1_p2)*100
       
        # Check If P2 is the final waypoint, detemine path complete
        if (self.waypoint_index_P2)==(self.number_of_waypoints-1):
            # set flag that final waypoint is in queqe
            self.reached_PN = True

            # If final waypoint reached (goal), set path complete to 100%
            if distance_to_goal <= self.distance_to_goal_tolerance:
                self.path_complete = float((self.waypoint_index_P2+1)/self.number_of_waypoints)*100
        
        # If P2 is not the final waypoint, update the points to track
        else:
            self.path_complete = float((self.waypoint_index_P2)/self.number_of_waypoints)*100

        
        # calculate distance to previous robot location
        self.distance_to_previous_pose = self.calculate_haversine_distance(
            [self.x_pose, self.y_pose], 
            [self.x_pose_prev, self.y_pose_prev])

        # set previous position for heading
        if self.distance_to_previous_pose > 0.1:
            # robot heading
            self.theta_robot = self.calculate_haversine_bearing(
                [self.x_pose_prev, self.y_pose_prev],
                [self.x_pose, self.y_pose])
            self.x_pose_prev = self.x_pose
            self.y_pose_prev = self.y_pose
            self.distance_to_previous_pose=0.0

        # declaring message components to publish
        #   path complete
        path_complete = self.path_complete

        #   Forward-Track-Error
        e_cg_x = dist_p2_cte

        #   Cross-Track-Error
        e_cg_y = -e_cg_y_cross

        #   Heading Error
        theta_path_diff = self.theta_path[self.waypoint_index_P2] - self.theta_path[self.waypoint_index_P1]
        theta_path_calc = np.mean([self.theta_path[self.waypoint_index_P1], self.theta_path[self.waypoint_index_P2]])
        if abs(theta_path_diff) > abs(theta_path_calc):
            theta_path_calc = self.theta_path[self.waypoint_index_P2]
        e_heading = (self.theta_pose - theta_path_calc)
        #       making correction when |error| greater than 270deg
        if abs(e_heading) > (3/2 * math.pi):
            e_heading = e_heading % (-np.sign(e_heading) * 2 * math.pi)
        e_heading=0.0

        # Debug info
        if self.show_logger:
            self.get_logger().info(
                f'\n Pose (lat,long):'
                f'\n                         robot: ({str(self.y_pose)}, {str(self.x_pose)})'
                f'\n                    robot prev: ({str(self.y_pose_prev)}, {str(self.x_pose_prev)})'
                f'\n                       path P1: ({str(self.P1[1])}, {str(self.P1[0])})'
                f'\n                       path P2: ({str(self.P2[1])}, {str(self.P2[0])})'
                f'\n                    e_cg_y (m): {str(e_cg_y)}'
                f'\n  '
                f'\n Heading (degrees):'
                f'\n                         robot: {str(math.degrees(self.theta_pose))}'
                f'\n                     path mean: {str(math.degrees(theta_path_calc))}'
                f'\n                       path P1: {str(math.degrees(self.theta_path[self.waypoint_index_P1]))}'
                f'\n                       path P2: {str(math.degrees(self.theta_path[self.waypoint_index_P2]))}'
                f'\n                    robot calc: {str(math.degrees(self.theta_robot))}'
                # f'\n                     path calc: {str(math.degrees(theta_path))}'
                f'\n                     e_heading: {str(math.degrees(e_heading))}'
                f'\n  '
                f'\n Points Reached:'
                f'\n          First waypoint (T/F): {str(self.reached_Pi)}'
                f'\n         Future waypoint (T/F): {str(self.reached_P2)}'
                f'\n          Final waypoint (T/F): {str(self.reached_PN)}'
                f'\n  '
                f'\n Path Complete:'
                f'\n               total waypoints: {str(self.number_of_waypoints)}'
                f'\n             waypoint_index_P1: {str(self.waypoint_index_P1)}'
                f'\n             waypoint_index_P2: {str(self.waypoint_index_P2)}'
                f'\n        lon distance to P1 (m): {str(dist_p1_cte)}'
                f'\n        lon distance to P2 (m): {str(dist_p2_cte)}'
                f'\n          distance to goal (m): {str(distance_to_goal)}'
                f'\n      distance to previous (m): {str(self.distance_to_previous_pose)}'
                f'\n    % path complete from P1-P2: {str(self.path_complete_p12)}%'
                f'\n               % path complete: {str(self.path_complete)}%'
                )
        return e_cg_x, e_cg_y, e_heading, path_complete

    def publish_error(self):
        try:
            self.distance_to_goal_tolerance = self.get_parameter('distance_to_goal_tolerance').value 
            self.reset_path = self.get_parameter('reset_path').value 
            self.waypoint_look_ahead = self.get_parameter('waypoint_look_ahead').value
            self.show_logger = self.get_parameter('show_logger').value 
            if self.path_received and self.initial_pose_set:
                longitdudinal_error, lateral_error, heading_error, path_complete = self.calculate_error_perp_path()
                self.error_msg.data = [float(longitdudinal_error), float(lateral_error), float(heading_error), float(path_complete)]
                self.error_publisher.publish(self.error_msg)
        except KeyboardInterrupt:
            pass

    def log_pose(self):
            self.get_logger().info(f'X Pose: {self.x_pose}, Y Pose: {self.y_pose}')

def main(args=None):
    rclpy.init(args=args)
    gps_error = GpsError()
    try:
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(gps_error)
        try:
            executor.spin()
        finally:
            gps_error.get_logger().info(f'Shutting down {NODE_NAME}...')
            time.sleep(1)
            executor.shutdown()
            gps_error.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
