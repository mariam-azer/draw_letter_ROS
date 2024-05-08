#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pow
import numpy as np
from tf.transformations import euler_from_quaternion

class GoToGoal:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  #10Hz
        self.goal = Point()
        self.current_pose = Point()
        self.rot_q = 0
        self.theta = 0.0
        
        self.arr = [0.01, 2, 1, 1, 2, 2, 2, 0.01]   #test
        self.current_pos = 0
	
        self.linear_tolerance = 0.09  # Tolerance for linear distance to the goal
        self.angular_tolerance = 0.2  # Tolerance for orientation to the goal
        
        self.kp_linear = 0.05  # Proportional gain for linear velocity     #first trial,0.5,           
        self.kp_angular = 0.09  # Proportional gain for angular velocity
        
        self.ki_linear = 0.01  # Integral gain for linear velocity
        self.ki_angular = 0.01  # Integral gain for angular velocity
        
        self.kd_linear = 0.01  # Derivative gain for linear velocity
        self.kd_angular = 0.05  # Derivative gain for angular velocity
        
        self.prev_error_linear = 0.0  # Previous error for linear velocity
        self.prev_error_angular = 0.0  # Previous error for angular velocity
        self.integral_linear = 0.0  # Integral term for linear velocity
        self.integral_angular = 0.0  # Integral term for angular velocity

    def update_pose(self, data):
        self.current_pose.x = data.pose.pose.position.x
        self.current_pose.y = data.pose.pose.position.y
        self.rot_q = data.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion ([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])
        if self.theta < 0:
        	self.theta = (np.pi * 2) - (self.theta * -1)

    def get_distance(self):
    	return sqrt(pow((self.goal.x - self.current_pose.x), 2) + pow(((self.goal.y) - self.current_pose.y), 2))

    def get_orientation(self):
        first_quad =  atan2(abs(self.goal.y - self.current_pose.y), abs(self.goal.x - self.current_pose.x)) 
        if((self.goal.y - self.current_pose.y) > 0) and ((self.goal.x - self.current_pose.x) > 0):
        	return first_quad
        elif ((self.goal.y - self.current_pose.y) > 0) and ((self.goal.x - self.current_pose.x) < 0):
        	return np.pi - first_quad
        elif ((self.goal.y - self.current_pose.y) < 0) and ((self.goal.x - self.current_pose.x) < 0):
        	return np.pi + first_quad
        elif ((self.goal.y - self.current_pose.y) < 0) and ((self.goal.x - self.current_pose.x) > 0):
        	return (np.pi * 2) - first_quad

    def move_to_goal(self):
    	while not rospy.is_shutdown():
    		self.goal.x = self.arr[self.current_pos]
    		self.goal.y = self.arr[self.current_pos + 1]
    		
    		distance_to_goal = self.get_distance()
    		
    		
    		
    		#if distance_to_goal > self.linear_tolerance:
    		desired_angle = self.get_orientation()
    		error_linear = distance_to_goal
    		error_angular = abs(desired_angle - self.theta)
    		self.integral_linear += error_linear * (0.1)
    		self.integral_angular += error_angular * (0.1)
    		
    		derivative_linear = abs(error_linear - self.prev_error_linear) / 0.1
    		derivative_angular = abs(error_angular - self.prev_error_angular) / 0.1
    		linear_velocity = self.kp_linear * error_linear + self.ki_linear * self.integral_linear + self.kd_linear * derivative_linear
    		angular_velocity = self.kp_angular * error_angular + self.ki_angular * self.integral_angular + self.kd_angular * derivative_angular
    		vel_msg = Twist()
    		print(f"{error_angular},     {desired_angle },   {self.theta}")
    		
    		if (linear_velocity > 0.1):
    			linear_velocity = 0.1
    
    		if angular_velocity < 0:
    			angular_velocity = angular_velocity * -1
    			
    		if (angular_velocity > 0.4):
    			angular_velocity = 0.4
    			
    			
    		
    		if error_angular > 0.15:
    			vel_msg.angular.z = angular_velocity
    			linear_velocity = 0.0
    		else:
    			vel_msg.linear.x = linear_velocity
    			vel_msg.angular.z = 0
    			
    		self.velocity_publisher.publish(vel_msg)
    		
    		self.prev_error_linear = error_linear
    		self.prev_error_angular = error_angular
    		#print(f"{desired_angle},   {self.goal.x},   {self.goal.y}  {self.current_pose.x}   {self.current_pose.y}")
    		#print(np.arctan[-0.02, -0.25])
    		
    		#else:
    		if (abs(self.current_pose.x - self.goal.x) < 0.08) and (abs(self.current_pose.y - self.goal.y) < 0.08) and ((len(self.arr) - 2) != self.current_pos):
    			self.current_pos = self.current_pos + 2
    		if (abs(self.current_pose.x - self.goal.x) < 0.08) and (abs(self.current_pose.y - self.goal.y) < 0.08) and ((len(self.arr) - 2) == self.current_pos):
    			vel_msg = Twist()
    			vel_msg.linear.x = 0.0
    			vel_msg.angular.z = 0.0
    			self.velocity_publisher.publish(vel_msg)
    			rospy.loginfo("Goal reached!")
    			#break
    		self.rate.sleep()


if __name__ == '__main__':
    try:
        bot = GoToGoal()
        #bot.goal.x = float(input("Enter goal x‐coordinate: "))
        #bot.goal.y = float(input("Enter goal y‐coordinate: "))
        bot.move_to_goal()
    except rospy.ROSInterruptException:
        pass
