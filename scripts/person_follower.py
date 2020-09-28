#!/usr/bin/env python3
from __future__ import print_function
import tty
import select
import sys
import termios
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
from math import pi
from sensor_msgs.msg import LaserScan
from scipy.ndimage import gaussian_filter1d


class PersonFollowNode(object):
    def __init__(self):
        rospy.init_node('person_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_angle = 0
        self.desired_distance = 0.5
        self.linear_k = -.5
        self.angular_k = -.05
        self.angular_vel = 0
        self.linear_vel = 0
        self.sigma = 1

    def process_laser(self, msg):
        distances = np.zeros(361)
        # Store all distance values in an array (index = angle)
        for angle, distance in enumerate(msg.ranges):
            if math.isinf(distance):
                distances[angle] = 1000
            else:
                distances[angle] = distance
        # Apply a gaussian filter to the distances
        filtered_data = gaussian_filter1d(np.array(distances), self.sigma)

        # Get the index of the minimum in the filtered lists
        min_index = np.where(filtered_data == min(filtered_data))
        current_distance = min(filtered_data)

        # Getting the first index with minimum distance (is an angle)
        current_angle = min_index[0][0]
        if current_angle > 180:
            current_angle = -(360 - current_angle)

        # print("current distance: ", current_distance, ",current_angle: ", current_angle)

        # Calculate error between current and desired position
        angular_error = self.desired_angle - current_angle
        linear_error = self.desired_distance - current_distance

        # Calculate velocities using proportional control
        self.angular_vel = self.angular_k * angular_error
        self.linear_vel = self.linear_k * linear_error
        print("Angular vel: ", self.angular_vel, ", Linear vel: ", self.linear_vel)


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.linear_vel), angular=Vector3(z=self.angular_vel)))
            r.sleep()


if __name__ == '__main__':
    wall = PersonFollowNode()
    wall.run()