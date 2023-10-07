#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class FeedbackControllerNode(Node):
    def __init__(self):
        super().__init__('feedback_controller_node')
        self.current_yaw = 0.0
        self.target_yaw = math.radians(90)  
        self.kp = 0.5  
        self.max_angular_vel = 1.0  
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        _, _, self.current_yaw = self.euler_from_quaternion(qx, qy, qz, qw)

    def euler_from_quaternion(self, x, y, z, w):
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

        return roll_x, pitch_y, yaw_z

    def calculate_control_command(self):
        yaw_error = self.target_yaw - self.current_yaw
        angular_vel_cmd = self.kp * yaw_error

        if angular_vel_cmd > self.max_angular_vel:
            angular_vel_cmd = self.max_angular_vel
        elif angular_vel_cmd < -self.max_angular_vel:
            angular_vel_cmd = -self.max_angular_vel

        return angular_vel_cmd

    def run_controller(self):
        rate = self.create_rate(10)  

        while rclpy.ok():
            angular_vel_cmd = self.calculate_control_command()
            twist = Twist()
            twist.angular.z = angular_vel_cmd
            self.vel_publisher.publish(twist)

            rate.sleep()

def main():
    rclpy.init(args=None)
    controller_node = FeedbackControllerNode()
    controller_node.run_controller()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
