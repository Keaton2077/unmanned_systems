#!/usr/bin/env python3
from re import S
import rclpy
import math 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import PIDTemplate

##RRT
import matplotlib.pyplot as plt
import numpy as np
import math as m
import matplotlib.patches as patches

class Node1:
    def __init__(self, x: float, y: float, cost: float, parent_idx: int) -> None:
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_idx = parent_idx

class Obstacle:
    def __init__(self, x_pos: float, y_pos: float, radius: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
        
    def is_inside(self, curr_x: float, curr_y: float) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        return dist_from <= self.radius

def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def get_all_moves(current_x: float, current_y: float, gs: float) -> list:
    move_list = []
    gs_x_bounds = np.arange(-gs, gs + gs, gs)
    gs_y_bounds = np.arange(-gs, gs + gs, gs)
    
    for dx in gs_x_bounds:
        for dy in gs_y_bounds:
            x_next = current_x + dx
            y_next = current_y + dy
            
            if [x_next, y_next] == [current_x, current_y]:
                continue
            
            move = [x_next, y_next]
            move_list.append(move)
            
    return move_list

def is_valid_move(obstacle_list: list, x_min: float, y_min: float, x_max: float, y_max: float, x_curr: float, y_curr: float, robot_radius: float) -> bool:
    if x_min > x_curr or x_max < x_curr or y_min > y_curr or y_max < y_curr:
        return False
    
    for obs in obstacle_list:
        if np.sqrt((obs.x_pos - x_curr)**2 + (obs.y_pos - y_curr)**2) <= obs.radius + robot_radius:
            return False
    
    return True

def compute_index(min_x: float, max_x: float, min_y: float, max_y: float, gs: float, x_current: float, y_current: float) -> int:
    index = int(((x_current - min_x) / gs) + (((y_current - min_y) / gs) * ((max_x + gs) - min_x) / gs))
    return index

def rrt(start_point, goal_point, gs, obstacle_positions, obstacle_radius, obstacle_list, max_iter=20000, step_size=1, robot_radius=0.5):
    min_x, max_x = 0, 15
    min_y, max_y = 0, 15
    
    fig, ax = plt.subplots()
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    
    for x in np.arange(min_x, max_x + gs, gs):
        for y in np.arange(min_y, max_y + gs, gs):
            index = compute_index(min_x, max_x, min_y, max_y, gs, x, y)
            ax.text(x, y, str(index), color='red', fontsize=6, ha='center', va='center')

    for obs_pos in obstacle_positions:
        obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_circle = patches.Circle((obstacle.x_pos, obstacle.y_pos), radius=obstacle.radius, edgecolor='black', facecolor='black')
        ax.add_patch(obstacle_circle)

    start_circle = patches.Circle((start_point[0], start_point[1]), radius=robot_radius, edgecolor='yellow', facecolor='none')
    ax.add_patch(start_circle)
    
    goal_circle = patches.Circle((goal_point[0], goal_point[1]), radius=robot_radius, edgecolor='blue', facecolor='none')
    ax.add_patch(goal_circle)

    tree = {start_point: None}

    for _ in range(max_iter):
        random_point = (np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y))
        nearest_point = min(tree, key=lambda p: euclidean_distance(p, random_point))
        
        # Move towards the random point with step size
        delta_x = random_point[0] - nearest_point[0]
        delta_y = random_point[1] - nearest_point[1]
        distance = euclidean_distance(nearest_point, random_point)
        if distance > step_size:
            scale = step_size / distance
            new_point = (nearest_point[0] + delta_x * scale, nearest_point[1] + delta_y * scale)
        else:
            new_point = random_point
        
        if is_valid_move(obstacle_list, min_x, min_y, max_x, max_y, new_point[0], new_point[1], robot_radius):
            tree[new_point] = nearest_point
            if euclidean_distance(new_point, goal_point) <= step_size:
                tree[goal_point] = new_point
                break
        
        if euclidean_distance(new_point, goal_point) <= step_size:
            tree[goal_point] = new_point
            print("Goal point reached:", goal_point)
            break
        
    if goal_point not in tree:
        print("Goal point not reached:", goal_point)
        return []

    # Reconstruct the path
    path = [goal_point]
    while path[-1] != start_point:
        path.append(tree[path[-1]])

    path.reverse()
    path_x = [point[0] for point in path]
    path_y = [point[1] for point in path]

    # Plot the tree
    for point, parent in tree.items():
        if parent is not None:
            plt.plot([point[0], parent[0]], [point[1], parent[1]], color='gray', linewidth=0.5)

    plt.plot(path_x, path_y, marker='o', color='green', linestyle='-')
    plt.show(block=False)
    plt.pause(1)
    return path

start_point = (1, 1)
goal_point = (7, 13)
gs = 1
Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 
8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7, 
8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 
14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
obstacle_positions = list(zip(Obstacle_x, Obstacle_y))
obstacle_radius = 0.5
obstacle_list = [Obstacle(obs_pos[0], obs_pos[1], obstacle_radius) for obs_pos in obstacle_positions]
robot_radius = 0.5

# Use the rrt function to find the path using RRT
wp_list = rrt(start_point, goal_point, gs, obstacle_positions, obstacle_radius, obstacle_list, robot_radius=robot_radius)



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
        
        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw
        
        print("yaw is", np.degrees(self.orientation_euler[2]))
        
    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
def main()->None:
    rclpy.init(args=None)
    print("starting")

    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)
    
    time_now = get_time_in_secs(turtlebot_node)
    print("time now is", time_now)

    kp_angular = 0.75
    ki_angular = 0.0
    kd_angular = 1.0
    dt_angular = 1/20
    pid_angular = PIDTemplate.PID(
        kp = kp_angular,
        ki = ki_angular,
        kd = kd_angular,
        dt = dt_angular
    )

    MAX_ANG_SPEED_RAD = 2.84

    heading_error_tol_rad = np.deg2rad(2)
    distance_error_tolerance_m = 0.15

    try:
        rclpy.spin_once(turtlebot_node)
        while rclpy.ok():
                
            for i in range(len(wp_list)):

                current_wp = wp_list[i] 

                dx = current_wp[0] - turtlebot_node.current_position[0]
                dy = current_wp[1] - turtlebot_node.current_position[1]
                desired_heading_rad = np.arctan2(dy,dx)

                current_heading_error_rad = pid_angular.compute_error(
                    desired_heading_rad,
                    turtlebot_node.orientation_euler[2]
                )

                current_distance_error = np.sqrt(dx**2 + dy**2)

                while abs(current_heading_error_rad) >= heading_error_tol_rad:
                    current_heading_error_rad = pid_angular.compute_error(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]
                    )

                    if (abs(current_heading_error_rad) <= heading_error_tol_rad):
                        print("I'm done")
                        break

                # print("desired heading error",np.rad2deg(current_heading_error_rad))
                    #print("desired heading is",np.rad2deg(desired_heading_rad))
                    #print("current position is" , turtlebot_node.current_position)

                    angular_gains = pid_angular.get_gains(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]
                    )

                    print("my heading error is",
                        np.rad2deg(pid_angular.error[0]))
                    
                    if angular_gains > MAX_ANG_SPEED_RAD:
                        angular_gains = MAX_ANG_SPEED_RAD
                    elif angular_gains <= -MAX_ANG_SPEED_RAD:
                        angular_gains = -MAX_ANG_SPEED_RAD     

                    turtlebot_node.move_turtle(0.0,angular_gains)       

                    rclpy.spin_once(turtlebot_node)

                while current_distance_error >= distance_error_tolerance_m:
                    current_heading_error_rad = pid_angular.compute_error(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]                
                    )

                    angular_gains = pid_angular.get_gains(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]
                    )

                    print("my heading error is", 
                        np.rad2deg(pid_angular.error[0]))
                    
                    if angular_gains >= MAX_ANG_SPEED_RAD:
                        angular_gains = MAX_ANG_SPEED_RAD
                    elif angular_gains <= -MAX_ANG_SPEED_RAD:
                        angular_gains = -MAX_ANG_SPEED_RAD

                    dx = current_wp[0] - turtlebot_node.current_position[0]
                    dy = current_wp[1] - turtlebot_node.current_position[1]
                    current_distance_error = np.sqrt(dx**2 + dy**2)

                    # if len(current_trajectory) > 1:
                    #     x_traj = [p[0] for p in current_trajectory]
                    #     y_traj = [p[1] for p in current_trajectory]
                    #     plt.plot(x_traj, y_traj, color='red')
                    #     plt.pause(0.01)

                    if (current_distance_error <= distance_error_tolerance_m):
                        print("converged to wp")
                        turtlebot_node.move_turtle(0.0, 0.0)
                        if i == len(wp_list)-1:
                            raise KeyboardInterrupt()
                        break
                    
                    turtlebot_node.move_turtle(0.15, angular_gains)

                    rclpy.spin_once(turtlebot_node)

    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0,0.0)

path_x = [point[0] for point in wp_list]
path_y = [point[1] for point in wp_list]

plt.plot(path_x, path_y, marker='o', color='green', linestyle='-')


plt.title("")
plt.xlabel("X")
plt.ylabel("Y")
plt.show(block=False)
plt.pause(1)

if __name__ == '__main__':
    """apply imported function"""
    main()