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
        self.linear_k = .05
        self.angular_k = .005
        self.angular_vel = 0
        self.linear_vel = 0
        self.sigma = 1
        self.target_x = 5
        self.target_y = 5
        self.target_angle = int(math.degrees(math.atan(self.target_y/self.target_x)))
        self.target_distance = math.sqrt(self.target_x**2 + self.target_y**2)
        self.target_weight = 10

    def process_odom(self, msg):
        # Get x and y position relative to the world origin
        global_x= msg.pose.pose.position.x
        global_y = msg.pose.pose.position.y

        # Difference between neato and target x, y
        diff_x = self.target_x - global_x
        diff_y = self.target_y - global_y

        # distance between neato and target, direct path
        # squaring means this will always be positive
        self.linear_error = math.sqrt((diff_x**2) + (diff_y**2))

        # Orientation of the neato according to global reference frame (odom)
        rotation = 180 - math.degrees(euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0])
        
        # Desired direction of the neato, in order to get to target, according to global reference frame
        vector_to_target = 180 + int(math.degrees(math.atan(global_y/global_x)))
        if vector_to_target > 360:
            vector_to_target = vector_to_target - 360
        
        # Angle the neato should travel in order to get to target
        self.angular_error = -(rotation - vector_to_target)
 

    def process_scan(self, msg):
        # distances = np.zeros(361)
        # target_distances = np.zeros(361)

        # target_distances[self.target_angle] = self.target_weight
        # # Store all distance values in an array (index = angle)
        # for angle, distance in enumerate(msg.ranges):
        #     if math.isinf(distance):
        #         distances[angle] = 100
        #     else:
        #         distances[angle] = distance

        # # Apply a gaussian filter to the distances
        # filtered_data = gaussian_filter1d(np.array(distances), self.sigma)
        # filtered_target = gaussian_filter1d(np.array(target_distances), self.sigma)
        # filtered_objective = np.add(filtered_data, filtered_target)
        # #print(filtered_objective)
        # # Get the index of the minimum in the filtered lists
        # max_distance = max(filtered_objective)
        # index = np.where(filtered_objective == max_distance)


        # # Getting the first index with minimum distance (is an angle)
        # current_angle = index[0][0]
        # if current_angle > 180:
        #     current_angle = -(360 - current_angle)

        # # print("current distance: ", current_distance, ",current_angle: ", current_angle)

        # # Calculate error between current and desired position
        # angular_error = self.target_angle
        # linear_error = self.target_distance
        # print(angular_error)
        # print(linear_error)

        # Calculate velocities using proportional control
        self.angular_vel = self.angular_k * self.angular_error
        self.linear_vel = self.linear_k * self.linear_error

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.linear_vel), angular=Vector3(z=self.angular_vel)))
            r.sleep()


if __name__ == '__main__':
    obstacle = ObstacleAvoidNode()
    obstacle.run()
