#!/usr/bin/env python3
from __future__ import print_function
import tty
import select
import sys
import termios
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from math import pi
from sensor_msgs.msg import LaserScan


class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_velocity = .5
        self.k = -2
        self.angular = 0

    def process_laser(self, msg):
        p1 = msg.ranges[315] 
        p2 = msg.ranges[225]
        print(p1, ", p2: ", p2)
        if np.isinf(p1): p1 = 1000
        if np.isinf(p2): p2 = 1000
        error = p1 - p2
        self.angular = self.k * error
        print(self.angular)


    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity), angular=Vector3(z=self.angular)))
            r.sleep()


if __name__ == '__main__':
    wall = WallFollowNode()
    wall.run()