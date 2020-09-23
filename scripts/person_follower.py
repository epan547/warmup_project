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


class PersonFollowNode(object):
    def __init__(self):
        rospy.init_node('person_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_velocity = .5
        self.desired_angle = 0
        self.desired_distance = 0.5
        self.k = -2
        self.angular = 0

    def process_laser(self, msg):
        distances = np.zeros(360)
        print(len(msg.ranges))
        for angle in range(len(msg.ranges)):
            if np.isinf(angle):
                distances[angle] = 1000
            else:
                distances[angle] = msg.ranges[angle]

        angular_error = self.desired_angle - actual_angle
        linear_error = self.desired_distance - actual_distance
        self.angular = self.k * error
        print(self.angular)


    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity), angular=Vector3(z=self.angular)))
            r.sleep()


if __name__ == '__main__':
    wall = PersonFollowNode()
    wall.run()