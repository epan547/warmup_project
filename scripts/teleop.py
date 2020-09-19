#!/usr/bin/env python3
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Initializing user input
settings = termios.tcgetattr(sys.stdin)
key = None

# Init ROS node
rospy.init_node('teleop')

# Creating publisher for drive
teleop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
linear_speed = .5
angular_speed = 100 * 2 * pi / 360

# Checking user input
while key != '\x03':
    key = getKey()
    if key == 'w':
    	print("forward")
    	vel_msg.angular.z = 0
    	vel_msg.linear.x = linear_speed
    elif key == 'a':
    	print("left")
    	vel_msg.linear.x = 0
    	vel_msg.angular.z = angular_speed
    elif key == 's':
    	print("down")
    	vel_msg.angular.z = 0
    	vel_msg.linear.x = -linear_speed
    elif key == 'd':
    	print("right")
    	vel_msg.linear.x = 0
    	vel_msg.angular.z = -angular_speed
    elif key == ' ':
    	print("stop")
    	vel_msg.angular.z = 0
    	vel_msg.linear.x = 0
    teleop_pub.publish(vel_msg) # send instructions to the robot
    rospy.Rate(10).sleep