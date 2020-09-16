#!/usr/bin/env python3
from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Twist, Vector3


rospy.init_node('marker')
vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
marker = Marker()
marker.header.frame_id = 'odom'
marker.type = marker.SPHERE
marker.scale.x = .5
marker.scale.y = .5
marker.scale.z = .5
marker.pose.position.x = 1
marker.pose.position.y = 2
marker.color.a = 1
marker.color.g = 1


vis_pub.publish(marker)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    # my_point_stamped.header.stamp = rospy.Time.now()    # update timestamp
    vis_pub.publish(marker)
    # print('hiiii')
    r.sleep()