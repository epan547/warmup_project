#!/usr/bin/env python3
from __future__ import print_function
import tty
import select
import sys
import termios
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3
import math
from sensor_msgs.msg import LaserScan


class WallFollowNode(object):
    def __init__(self):
        # Initialize the ROS node, and equation variables
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_velocity = .5
        self.desired_distance = .5
        self.kp = 2
        self.ki = 1
        self.kd = 2
        self.angular = 0
        self.error_prior = 0
        self.integral_prior = 0
        self.itertime = .1

    def process_laser(self, msg):
        # Get the distance of two points
        d1 = msg.ranges[315] 
        d2 = msg.ranges[225]
        if np.isinf(d1): d1 = 1000
        if np.isinf(d2): d2 = 1000
        distance = (d1/math.sqrt(2) + d2/math.sqrt(2))/2
        error = self.desired_distance - distance

        integral = self.integral_prior + error * self.itertime
        derivative = (error - self.error_prior)/self.itertime
        self.angular = self.kp*error + self.ki*integral + self.kd*derivative 

        self.error_prior = error
        self.integral_prior = integral

        print(error)


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity), angular=Vector3(z=self.angular)))
            r.sleep()


if __name__ == '__main__':
    wall = WallFollowNode()
    wall.run()