#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import os
import numpy as np

position_ = Point()


def clbk_odom(msg):
	global position_
	position_ = msg.pose.pose.position
    
def stop():
    print 'Killing nodes'

    nodes = os.popen("rostopic info /cmd_vel").readlines()
    pub_nodes = []
    for i in range(3,len(nodes)):
        if nodes[i] == '\n':
            break
        pub_nodes.append(nodes[i].split()[1])
    #     nodes[i] = nodes[i].replace("\n","")

    for node in pub_nodes:
        os.system("rosnode kill "+ node)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


rospy.init_node('kill_switch')
sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, clbk_odom)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if np.hypot(position_.x - 1.176, position_.y - -0.268) <=0.1:
        stop()
    rate.sleep()
