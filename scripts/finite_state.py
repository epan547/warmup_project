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
from tf.transformations import euler_from_quaternion

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
        self.angular_k = .005
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
            # send instructions to the neato
            self.vel_pub.publish(self.vel_msg) 
            rospy.Rate(10).sleep


    def square(self):
        for i in range(4):
            self.vel_pub.publish(Twist(linear=Vector3(x=1)))
            rospy.sleep(2)
            self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
            rospy.sleep(2)
        #make sure it's not moving when it goes back to teleop
        self.vel_msg.angular.z = 0
        self.vel_msg.linear.x = 0
        self.vel_pub.publish(self.vel_msg)
        self.state = "teleop"


    def process_odom(self,msg):
        #get our x and y position relative to the world origin"
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Orientation of the neato according to global reference frame (odom)
        self.rotation = 180 - math.degrees(euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0])
        # distance between neato and target
        self.linear_error = math.sqrt(self.x**2 + self.y**2)
        # Desired direction of the neato, in order to get to target, according to global reference frame
        self.vector_to_target = 180 + int(math.degrees(math.atan(self.y/self.x)))
        if self.vector_to_target > 360:
            self.vector_to_target = self.vector_to_target - 360
        self.angular_error = -(self.rotation - self.vector_to_target)

    def origin(self):
        # Navigates the neato towards the origin of the global / odometry coordinate system
        if self.linear_error < 1:
            self.state = "teleop"
        else:
            # Apply proportional control constants
            self.angular_vel = self.angular_k * self.angular_error
            self.linear_vel = self.linear_k * self.linear_error
            # send instructions to the robot
            self.vel_pub.publish(Twist(linear=Vector3(x=self.linear_vel), angular=Vector3(z=self.angular_vel)))
            #print("Angular velocity: ", self.angular_vel, ", Linear velocity: ", self.linear_vel)
            rospy.Rate(10).sleep

if __name__ == '__main__':
    node = NeatoController()
    node.run()
