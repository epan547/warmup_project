#!/usr/bin/env python3
from __future__ import print_function
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi
from sensor_msgs.msg import LaserScan

# def callback(msg):
#     print(len(msg.ranges))

# rospy.init_node('wall_follow')
# sub = rospy.Subscriber('/scan', LaserScan, callback)

# rospy.spin()

class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_velocity = .5
        self.k = -2
        self.angular = 0

    def process_laser(self, msg):
        error = msg.ranges[315] - msg.ranges[245]
        # print("Angle 1: ", msg.ranges[315], ", Angle 2:", msg.ranges[245])
        # if error > 0:
        #     vel_msg.angular.z = -k * error
        # elif error < 0:
        #     vel_msg.angular.z = k * error
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