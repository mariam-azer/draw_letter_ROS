#!/usr/bin/env python3
import rospy
import numpy as num
import matplotlib.pyplot as plot
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

x = 0.0
y = 0.0


def callback(msg):
    	
    global x
    global y
   
	
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
	
    
    	
class DifferentialDriveRobot:
    def __init__ (self):
       
        self.x =[] #@initial x position
        self.y=[] #e initial y position
     
        
    def move(self, x, y):#Compute wheel speeds

            #Update robot pose
            self.x.append(x)
            self.y.append(y)
          
            
    def plot_robot(self):
        #Plot robot
        plot.plot(self.x,self.y,'ro')
        #plot.quiver(self.x,self.y, num.cos(self.theta), num.sin(self.theta))
        plot.axis('equal')
        plot.grid(True)
        
def listener():
    
    rospy.init_node('plotting', anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback)
    robot = DifferentialDriveRobot()
    while not rospy.is_shutdown():
        robot.move(x, y)
        robot.plot_robot()
        plot.draw()
        plot.pause(0.1)
        
if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
        	pass
