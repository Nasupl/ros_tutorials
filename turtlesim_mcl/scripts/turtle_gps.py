#!/usr/bin/env python
# vim:fileencoding=utf-8

import rospy
from turtlesim.msg import *
from numpy.random import *

def pos_cb(data):
    pub_pose = data

    pub_pose.x = pub_pose.x + normal(1.0, 0.1)
    pub_pose.y = pub_pose.x + normal(1.0, 0.1)
    pub_pose.theta = pub_pose.theta + normal(1.0, 0.05)

    pub = rospy.Publisher('/turtle1/gps', Pose, queue_size=10)
    pub.publish(pub_pose)

    rospy.sleep(1.0)

def listener():
    rospy.init_node('turtle_gps')
    rospy.Subscriber('/turtle1/pose', Pose, pos_cb, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
