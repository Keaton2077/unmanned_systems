#!/usr/bin/env python3
from re import S
import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.duration  import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import PIDTemplate

from sensor_msgs.msg import LaserScan
from unmanned_systems_ros2_pkg import quaternion_tools, PIDTemplate

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from unmanned_systems_ros2_pkg import quaternion_tools, PIDTemplate
from unmanned_systems_ros2_pkg.d_star_lite import DStarLite

from pathfinding_lib import Djikstras

def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9 
    
def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
def get_mean_target(heading_list:list)-> float:
     heading_list = np.array(heading_list)
     mean_heading_target = np.mean(heading_list)
     return mean_heading_target 

def get_mean_dist(dist_list:list)-> float:
    dist_list = np.array(dist_list)
    mean_dist = np.mean(dist_list)
    return mean_dist
def compute_global_heading(heading_target_rad:float, 
                        curent_yaw_rad:float):
    
    global_heading_rad = heading_target_rad + curent_yaw_rad
    
    if global_heading_rad > 2*np.pi:
        global_heading_rad = global_heading_rad - 2*np.pi
    elif global_heading_rad < 0:
        global_heading_rad = global_heading_rad + 2*np.pi
    
    print("global heading deg", np.rad2deg(global_heading_rad))
    
    return global_heading_rad
def get_all_targets(heading_list:list)-> float:
    heading_list = np.array(heading_list)

    for j in range(5):
        heading_slice_list.append(heading_list(j*i))

    for i in range(1,76):
        heading_slice_list = []
        for j in range(1,6):
            heading_slice_list.append(heading_list(j*i))
    mean_heading_target = np.mean(heading_list)



        
    mean_heading_target = np.mean(heading_list)
    return mean_heading_target 

class TurtleBotNode(Node):
    def __init__(self, ns=''):
        super().__init__('minimial_turtlebot')
        
        if ns != '':
            self.ns = ns
        else:
            self.ns = ns
                
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  10) 
        
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 10)
        
        self.lidar_subscriber = self.create_subscription(
             LaserScan, self.ns+"/scan", self.lidar_track_cb, 1)
        

        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw
        
        self.current_velocity = [0.0, 0.0]
        
        self.evader_position = [0,0]
        self.evader_velocity = [0,0]
        
        self.detected_range_list = [] #depth detected
        self.detected_heading_angle_list = [] #heading detected


        
        
    def odom_callback(self,msg:Odometry):
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = quaternion_tools.euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw
        
        current_vel_mag = msg.twist.twist.linear.x
        self.current_velocity[0] = current_vel_mag*np.cos(yaw)
        self.current_velocity[1] = current_vel_mag*np.sin(yaw)
        
        
    def move_turtle(self, linear_vel:float, angular_vel:float):
        """Moves turtlebot"""
        twist = Twist()
        
        if linear_vel >= 0.23:
            linear_vel = 0.23
        elif linear_vel <= -0.23:
            linear_vel = -0.23
        
        if angular_vel >= 2.84:
            angular_vel = 2.84
        elif angular_vel <= -2.84: 
            angular_vel = -2.84    
    
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
    def lidar_track_cb(self, msg:LaserScan):
        """lidar information remember the msg is an array of 0-> 359"""
        self.detected_range_list = []
        self.detected_heading_angle_list = []
        inf = float('inf')

        lidar_vals = msg.ranges
        
        #append detections if values not infinity or 0.0 
        for i, val in enumerate(lidar_vals):

            if val != inf:
                self.detected_heading_angle_list.append(i)
                self.detected_range_list.append(val)


def tb_nav_final(start_x,start_y,min_x,max_x,min_y,max_y,goal_x,goal_y,gs,obstacle_positions,obstacle_radius, iter):

#def main()->None:
    #if iter[0] == 0 and iter[1] == 0:
    rclpy.init(args=None)
    
    print("starting")

    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)

    rate = turtlebot_node.create_rate(rate_val)
    

    
    # time intilization ref 
    time_origin = get_time_in_secs(turtlebot_node)
    print("time now is", time_origin)

    kp_angular = 1
    ki_angular = 0.0
    kd_angular  = 0.0
    dt_angular = 1/20

    pid_angular = PIDTemplate.PID(
        kp = kp_angular,
        ki = ki_angular,
        kd = kd_angular,
        dt = dt_angular)

    MAX_ANG_SPEED_RAD = 2.84 #rad/s
    #start_x = 0
    #start_y = 0

    #min_x = 0
    #max_x = 10

    #min_y = 0
    #max_y = 10

    #goal_x = 9
    #goal_y = 9

    #gs = 1.0

    #obstacle_positions =  [(5,0),(5,1),(5,2),(5,3),(5,4),(0,5),(1,4),(2,3),(3,2),(3,3)]
    #obstacle_positions = [] # store obstacle classes
    #obstacle_radius = 0.5

    #path =  djikstras(start_x, start_y, min_x, min_y, max_x,max_y, 
    #        obstacle_positions, gs, obstacle_radius,goal_x,goal_y,obstacle_list)
    
    start_x = round(start_x/gs)*gs
    start_y = round(start_y/gs)*gs

    path = Djikstras.Djikstras.djikstras(min_x,max_x,min_y,max_y,obstacle_positions,obstacle_radius,-3.5,-3.5,goal_x,goal_y,gs,0.0)
    #if path:
    #    path.pop(0)
    path_x = path[0]
    path_y = path[1]
    path_len = len(path_x)
    path = []
    for i in range(path_len):
        path.append((path_x[path_len-i-1],path_y[path_len-i-1]))
    #print (path)
    if path:
        path.pop(0)
    wp_list = path
    init_wp_list = wp_list


    print("wp list", wp_list)

    heading_error_tol_rad = np.deg2rad(5)
    distance_error_tolerance_m = 0.25#m
    num_waypoints = len(wp_list)
    wp_counter = 0
    path_traversed = []
    current_pos = (start_x,start_y)
    obstacle_positions_tot = []
    current_pos_init = current_pos
    # try:
    try: 
        rclpy.spin_once(turtlebot_node)



        while rclpy.ok():

            #print("goal x",goal_x)
            #print("current pos 1", current_pos)
            #if iter == 100:
            #    obstacle_positions = []
            #    iter = 0
            if current_pos[0] == None:
                current_pos = current_pos_init
            #print("current pos", current_pos)
            goal_dist = np.sqrt((current_pos[0]-goal_x)**2 + (current_pos[1]-goal_y)**2)

            # if close to goal, stop:
            if goal_dist < distance_error_tolerance_m:
                #print("wp counter", wp_counter)
                #print("num_waypoints", num_waypoints)
                #print("path traversed", path_traversed)
                path_traversed_final = path_traversed
                print ("reached goal")
                turtlebot_node.move_turtle (0.0,0.0)
                return obstacle_positions_tot, turtlebot_node.current_position, path_traversed_final, init_wp_list
            
            print("curent pos", current_pos)
            if current_pos != current_pos_init:
                path_traversed.append((current_pos[0],current_pos[1]))
            #print("path traversed", path_traversed)
            #print("current obs list", obstacle_positions)

            temp_start_x = np.round(current_pos[0]/gs)*gs
            temp_start_y = np.round(current_pos[1]/gs)*gs

            try:
                path = DStarLite(min_x,max_x,min_y,max_y,obstacle_positions,obstacle_radius,temp_start_x,
                                                    temp_start_y,goal_x,goal_y,gs,0.0)
            except:
                obstacle_positions = []


            #if path:
            #    path.pop(0)

            path_x = path[0]
            path_y = path[1]
            path_len = len(path_x)
            path = []
            for i in range(path_len):
                path.append((path_x[path_len-i-1],path_y[path_len-i-1]))
            #print("current path", path)
            if path[0] == (goal_x,goal_y):
                path.append((goal_x,goal_y))
            current_wp = path[1]
            #path_len_temp = len(path)
            #current_wp = path[path_len_temp-1]

            if len(path) < 5:
                current_wp = (goal_x,goal_y)



            # get current waypoint
            #current_wp = wp_list[0]
            print("current wp:", current_wp)
            #print("current pos:", turtlebot_node.current_position)

            dx = current_wp[0] - current_pos[0]
            dy = current_wp[1] -  current_pos[1]

            #dx = current_pos[0]-current_wp[0]
            #dy = current_pos[1] - current_wp[1]
            desired_heading_rad = np.arctan2(dy,dx)

            

            desired_heading_rad = math.atan2((current_wp[1]-current_pos[1]),(current_wp[0]-current_pos[0]))
            #desired_heading_rad = np.arctan2((current_wp[0], current_wp[1]),(current_pos[0],current_pos[1]))

            #desired_heading_rad = -desired_heading_rad

            print("desired heading", np.rad2deg(desired_heading_rad))
            print("current heading", np.rad2deg(turtlebot_node.orientation_euler[2]))


            current_distance_error = np.sqrt(dx**2 + dy**2)



        
            ### SET CORRECT HEADING ------------


            
            current_heading_error_rad = pid_angular.compute_error(
                desired_heading_rad,
                turtlebot_node.orientation_euler[2]                
            )


            #print("current wp", current_wp)
            #checking lidar for obstacles
            # getting obs heading from lidar

            detected_heading_list = turtlebot_node.detected_heading_angle_list
            mean_target = np.deg2rad(get_mean_target(detected_heading_list))

            #print("detected heading list", detected_heading_list)
            
            #if an obstacle is detected:
            obstacle_positions = []
            if (math.isnan(mean_target)) == False:
                detected_range_list = turtlebot_node.detected_range_list
                j = 0
                for i in range(len(detected_heading_list)):
                    j = j + 1
                    if j == 5:
                        j = 0
                            

                        #adjusting heading for current TB LOS
                        global_mean_target = np.deg2rad(detected_heading_list[i]) + turtlebot_node.orientation_euler[2]
                        #print("mean detected heading", mean_target)
                        #getting obs distance from TB
                        mean_dist = detected_range_list[i]

                        #print("mean_target", mean_target)
                        #print("current heading", turtlebot_node.orientation_euler[2])


                        #estimating obstacle position, and rounding it to desired gs
                        #temp_obs_x = np.round((current_pos[0] + mean_dist * np.cos(global_mean_target))/gs)*gs
                        #temp_obs_y = np.round((current_pos[1] + mean_dist * np.sin(global_mean_target))/gs)*gs

                        temp_obs_x = current_pos[0] + mean_dist * np.cos(global_mean_target)
                        temp_obs_y = current_pos[1] + mean_dist * np.sin(global_mean_target)                  

                        obs_temp = (temp_obs_x,temp_obs_y)
                        #print("obs pos", temp_obs_x,temp_obs_y)

                        #if obs_temp not in obstacle_positions:
                            #continue
                        #else:
                        obs_goal_dist = math.dist(obs_temp,(goal_x,goal_y))
                        if obs_goal_dist > 0.1:
                            obstacle_positions.append(obs_temp)
                        obs_temp = (np.round((obs_temp[0]/gs))*gs,np.round((obs_temp[1]/gs))*gs)
                        if obs_temp not in obstacle_positions_tot:
                            obstacle_positions_tot.append(obs_temp)
                            
                            #print("new obs list:", obstacle_positions)

                #return obstacle_positions, turtlebot_node.current_position, path_traversed

                    

                    

                #if mean_dist > 0.0:
                    #new_obstacle_x = mean_dist * np.cos(mean_target)  
                    #new_obstacle_y = mean_dist * np.sin(mean_target)  
                    #is_obstacle = True            


            #print("curent pos", turtlebot_node.current_position)


            #print("my heading error is", 
            #    np.rad2deg(pid_angular.error[0]))
            
            angular_gains = pid_angular.get_gains(
                desired_heading_rad,
                turtlebot_node.orientation_euler[2]
            )

            print("current heading error", np.rad2deg(current_heading_error_rad))

            if angular_gains >= MAX_ANG_SPEED_RAD:
                angular_gains = MAX_ANG_SPEED_RAD
            elif angular_gains <= -MAX_ANG_SPEED_RAD:
                angular_gains = -MAX_ANG_SPEED_RAD
            

            print("angular gains", angular_gains)
            #angular_gains = -angular_gains
            
            dx = current_wp[0] - current_pos[0]
            dy = current_wp[1] -  current_pos[1]
            current_distance_error = np.sqrt(dx**2 + dy**2)
                
            if (np.abs(current_heading_error_rad) <= heading_error_tol_rad):
                print("converged to wp")
                turtlebot_node.move_turtle(0.15, 0.0)



            else:
                turtlebot_node.move_turtle(0.15, angular_gains)

            

            #print("current wp", current_wp)

            #turtlebot_node.move_turtle(0.15, angular_gains)
            #print("wp_counter", wp_counter)
            rclpy.spin_once(turtlebot_node)
            if turtlebot_node.current_position[0] != None:
                current_pos = ((np.round(turtlebot_node.current_position[0]/gs)*gs),(np.round(turtlebot_node.current_position[1]/gs)*gs))

            #iter = iter + 1
            #wp_counter = wp_counter + 1

    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0, 0.0)
    return
    

#if __name__ == '__main__':
#    """apply imported function"""
#    main()
