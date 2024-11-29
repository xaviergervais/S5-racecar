#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
import tf_transformations

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class PathFollowing(Node):
    def __init__(self):
        super().__init__('path_following')
        self.angle_div = self.declare_parameter('angle_div', 8).value
        self.distance = self.declare_parameter('distance', 0.7).value
        self.distance_short = self.declare_parameter('distance_short', 0.45).value
        self.max_speed = self.declare_parameter('max_speed', 1).value
        self.max_steering = self.declare_parameter('max_steering', 0.37).value

        self.goal_pub = self.create_publisher(PoseStamped, 'goal', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
	
        self.navigator = BasicNavigator()
        
        self.yaw = -90.0*np.pi/180.0
        self.quat = tf_transformations.quaternion_from_euler(0.0,0.0,self.yaw)   
	
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'racecar/map'
        self.goal_pose.pose.position.x = 2.1
        self.goal_pose.pose.position.y = -13.5
        self.goal_pose.pose.orientation.x = self.quat[0]
        self.goal_pose.pose.orientation.y = self.quat[1]
        self.goal_pose.pose.orientation.z = self.quat[2]
        self.goal_pose.pose.orientation.w = self.quat[3]
        
        self.navigator.goToPose(self.goal_pose)
        self.flag_reached = False
        self.flag_turnaround = False
        self.flag_timeout = False
        self.delay_sec = 0.5
        self.degrees = 30.0
        self.old_time = self.get_clock().now()
        self.delay_turnaround = 10.0     

    def scan_callback(self, msg):
        
        self.current_time = self.get_clock().now()
        #self.feedback = self.navigator.getFeedback()
        #self.distance_remaining = self.feedback.distance_remaining
        #self.get_logger().info(f"Distance: %.2f metres", self.distance_remaining)
        
        
        if not self.navigator.isTaskComplete() and not self.flag_reached:
            self.get_logger().info("Ce n'est pas atteint")
            if (self.current_time - self.old_time).nanoseconds / 1e9 > self.delay_sec:
                self.navigator.goToPose(self.goal_pose)
                self.old_time = self.get_clock().now()
        
        self.result = self.navigator.getResult()
        if self.result == TaskResult.SUCCEEDED and not self.flag_reached:
            self.navigator.cancelTask()
            self.navigator.clearAllCostmaps()
            self.get_logger().info("Premier atteint bot")
            self.flag_reached = True
            self.goal_pose = PoseStamped()
            self.goal_pose.header.frame_id = 'racecar/map'
            self.goal_pose.pose.position.x = 0.1
            self.goal_pose.pose.position.y = 0.1
            
            self.goal_pose.pose.orientation.x = -self.quat[0]
            self.goal_pose.pose.orientation.y = -self.quat[1]
            self.goal_pose.pose.orientation.z = -self.quat[2]
            self.goal_pose.pose.orientation.w = -self.quat[3]
            
            
       	elif self.flag_reached and not self.flag_turnaround:
       	    self.turn_around()
       	    
       	    
        elif self.result == TaskResult.FAILED:
            self.get_logger().info("Failure")
        elif self.result == TaskResult.CANCELED and not self.flag_reached:
            self.get_logger().info("Cancel culture")
        elif self.result == TaskResult.UNKNOWN:
            self.get_logger().info("Ignorant fucker")
        elif self.result == TaskResult.FAILED:
            self.get_logger().info("Failure")
            	   
           


    def odom_callback(self, msg):
        self.get_logger().info('Current speed: %f' % msg.twist.twist.linear.x)
        
    def turn_around(self):
    	
    	if not self.flag_timeout:
            self.old_time_turn = self.get_clock().now()
            self.get_logger().info('Intialisation turn around')
            self.flag_timeout = True
    		
    	twist = Twist()
    	twist.linear.x = -0.2
    	twist.linear.y = 0.0
    	twist.linear.z = 0.0
        
    	twist.angular.x = 0.0
    	twist.angular.y = 0.0
    	twist.angular.z = np.pi*self.degrees/180.0
        
    	self.cmd_vel_pub.publish(twist)
        
    	if (self.get_clock().now() - self.old_time_turn).nanoseconds / 1e9 > self.delay_turnaround:
    	    self.flag_turnaround = True
    	    self.flag_reached = False
    	    self.navigator.goToPose(self.goal_pose)
    	    self.get_logger().info('Je sais tourner sur moi meme caliss')
    	    self.result == TaskResult.UNKNOWN
    	    return
        
        
def main(args=None):
    rclpy.init(args=args)
    path_following = PathFollowing()
    rclpy.spin(path_following)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
