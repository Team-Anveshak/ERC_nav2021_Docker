#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import os

nodes = os.popen("rostopic info /cmd_vel").readlines()
pub_nodes = []
for i in range(3,len(nodes)):
    if nodes[i] == '\n':
        break
    pub_nodes.append(nodes[i].split()[1])
#     nodes[i] = nodes[i].replace("\n","")

for node in pub_nodes:
    os.system("rosnode kill "+ node)

rospy.init_node('kill_switch')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist_msg = Twist()
twist_msg.linear.x = 0
twist_msg.angular.z = 0
pub.publish(twist_msg)