#!/usr/bin/env python3
from __future__ import print_function
import tty
import select
import sys
import termios
import rospy
import numpy as np
import math
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Vector3, PoseWithCovariance
from nav_msgs.msg import Odometry
from math import pi
from sensor_msgs.msg import LaserScan
from scipy.ndimage import gaussian_filter1d
from visualization_msgs.msg import Marker


class ObstacleAvoidNode(object):
    def __init__(self):
        rospy.init_node('obstacle_avoid')
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_angle = 0
        self.desired_distance = 0
        self.linear_k = 0.001
        self.angular_k = -.05
        self.angular_vel = 0
        self.linear_vel = 0
        self.sigma = 1
        self.target_x = 15
        self.target_y = 15
        self.target_angle = int(math.degrees(math.atan(self.target_y/self.target_x)))
        self.target_distance = math.sqrt(self.target_x**2 + self.target_y**2)
        #self.vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        #self.arrow_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        # self.marker = self.create_marker()
        self.target_weight = 10
        # self.arrow = self.create_arrow()

    def process_odom(self, msg):
        x_remaining = self.target_x - msg.pose.pose.position.x
        y_remaining = self.target_y - msg.pose.pose.position.y
        self.orientation = math.degrees(msg.pose.pose.orientation.z)
        # self.target_angle = int(math.degrees(math.atan2(y_remaining, x_remaining)))
        self.target_angle = int(math.degrees(math.atan2(self.target_y, self.target_x))) - int(self.orientation)
        self.target_distance = math.sqrt(x_remaining**2 + y_remaining**2)
        #self.marker.pose.position.x = self.target_x
        #self.marker.pose.position.y = self.target_y

    # def create_marker(self):
    #     marker = Marker()
    #     marker.header.frame_id = 'odom'
    #     marker.type = marker.SPHERE
    #     marker.scale.x = .5
    #     marker.scale.y = .5
    #     marker.scale.z = .5
    #     marker.pose.position.x = self.target_x
    #     marker.pose.position.y = self.target_y
    #     marker.color.a = 1
    #     marker.color.g = 1
    #     return marker

    # def create_arrow(self):
        # marker = Marker()
        # marker.header.frame_id = 'base_link'
        # marker.type = marker.ARROW
        # marker.scale.x = self.target_distance
        # marker.scale.y = .2
        # marker.scale.z = .2
        # marker.pose.orientation.y = self.target_angle
        # marker.color.a = 1
        # marker.color.g = 1
        # marker.points = [tail, tip]
        # return marker

    def process_scan(self, msg):
        distances = np.zeros(361)
        target_distances = np.zeros(361)

        target_distances[self.target_angle] = self.target_weight
        # Store all distance values in an array (index = angle)
        for angle, distance in enumerate(msg.ranges):
            if math.isinf(distance):
                distances[angle] = 100
            else:
                distances[angle] = distance

        # Apply a gaussian filter to the distances
        filtered_data = gaussian_filter1d(np.array(distances), self.sigma)
        filtered_target = gaussian_filter1d(np.array(target_distances), self.sigma)
        filtered_objective = np.add(filtered_data, filtered_target)
        #print(filtered_objective)
        # Get the index of the minimum in the filtered lists
        max_distance = max(filtered_objective)
        index = np.where(filtered_objective == max_distance)


        # Getting the first index with minimum distance (is an angle)
        current_angle = index[0][0]
        if current_angle > 180:
            current_angle = -(360 - current_angle)

        # print("current distance: ", current_distance, ",current_angle: ", current_angle)

        # Calculate error between current and desired position
        angular_error = self.target_angle
        linear_error = self.target_distance
        print(angular_error)
        print(linear_error)
        # print(current_distance)
        # Calculate velocities using proportional control
        self.angular_vel = self.angular_k * angular_error
        self.linear_vel = self.linear_k * linear_error
        #print("Angular vel: ", self.angular_vel, ", Linear vel: ", self.linear_vel)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.linear_vel), angular=Vector3(z=self.angular_vel)))
            # self.vis_pub.publish(self.marker)
            # self.arrow_pub.publish(self.arrow)
            r.sleep()


if __name__ == '__main__':
    obstacle = ObstacleAvoidNode()
    obstacle.run()
