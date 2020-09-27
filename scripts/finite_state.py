#!/usr/bin/env python3
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3
import math
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class NeatoController():
    "This class encompasses multiple behaviors for the simulated neato"
    def __init__(self):
        rospy.init_node('finite_state')
        self.distance_threshold = 0.8
        self.state = "teleop"
        self.x = 0
        self.y = 0
        self.rotation = 0
        self.linear_error = 0
        self.angular_error = 0
        self.linear_k = 0.1
        self.angular_k = math.pi/2
        self.vel_msg = Twist()
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.process_scan)
        rospy.Subscriber('odom', Odometry, self.process_odom)
        # Initializing user input
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None

    def run(self):
        """ The run loop repeatedly executes the current state function.  Each state function will return a function
        corresponding to the next state to run. """
        # this sleep is to allow any subscribers to cmd_vel to establish a connection to our publisher.  This is only
        # needed in the case where you send the velocity commands once (in some ways sending the commands repeatedly is
        # more robust.
        rospy.sleep(1)
        while not rospy.is_shutdown():
            if self.state == "teleop":
                self.teleop()
            if self.state == "square":
                self.square()
            if self.state == "origin":
                self.origin()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def teleop(self):
        # Checking user input
        if self.key != '\x03':
            self.key = self.getKey()
            if self.key == 'w':
            	self.vel_msg.angular.z = 0
            	self.vel_msg.linear.x = 1
            elif self.key == 'a':
            	self.vel_msg.linear.x = 0
            	self.vel_msg.angular.z = 1
            elif self.key == 's':
            	self.vel_msg.angular.z = 0
            	self.vel_msg.linear.x = -1
            elif self.key == 'd':
            	self.vel_msg.linear.x = 0
            	self.vel_msg.angular.z = -1
            elif self.key == ' ':
            	self.vel_msg.angular.z = 0
            	self.vel_msg.linear.x = 0
            elif self.key == '1':
                self.vel_msg.angular.z = 0
                self.vel_msg.linear.x = 0
                self.state = "square"
            elif self.key == '2':
                self.vel_msg.angular.z = 0
                self.vel_msg.linear.x = 0
                self.state = "origin"
            self.vel_pub.publish(self.vel_msg) # send instructions to the robot
            rospy.Rate(10).sleep


    def square(self):
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        self.vel_msg.angular.z = 0
        self.vel_msg.linear.x = 0
        self.vel_pub.publish(self.vel_msg)
        self.state = "teleop"

    def process_scan(self,msg):
        pass

    def process_odom(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.rotation = math.degrees(msg.pose.pose.orientation.z)
        self.linear_error = math.sqrt(self.x**2 + self.y**2)
        self.angular_error = int(math.degrees(math.atan(self.y/self.x)))
        print(self.rotation)

    def origin(self):
        if self.linear_error < 0.25:
            self.state = "teleop"
        else:
            self.vel_msg.angular.z = self.angular_k * self.angular_error
            self.vel_msg.linear.x = self.linear_k * self.linear_error
            self.vel_pub.publish(self.vel_msg) # send instructions to the robot
            rospy.Rate(10).sleep

if __name__ == '__main__':
    node = NeatoController()
    node.run()
