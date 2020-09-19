#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi


# Init ROS node
rospy.init_node('drive_square')

# Creating publisher for drive
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

linear_speed = .5
angular_speed = pi/4

while not rospy.is_shutdown():

	pub.publish(Twist(linear=Vector3(x=linear_speed)))
	rospy.sleep(2)

	pub.publish(Twist(angular=Vector3(z=-angular_speed)))
	rospy.sleep(2)