#! /usr/bin/env python

# import ros stuff
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry


# rospy.init_node('wayp')
# pub = rospy.Publisher('/waypoint', Marker, queue_size=1)

# line = Marker()
# line.type = line.LINE_LIST
# line.scale.x = 0.2
# p = Point()
# p.x = 5
# p.y = 5
# p.z = 0
# line.points.append(p)
# p.z += 10
# line.points.append(p)
# line.header.frame_id = '/base_link'
# line.pose.orientation.w = 1.0
# line.action = line.ADD
# line.color.r = 1.0
# line.color.a = 1.0

# rate = rospy.Rate(10)
# while not rospy.is_shutdown():
#     pub.publish(line)
#     rate.sleep()

goal = Point()
pose = Point()

def clk_goal(msg):
    global goal
    goal.x = msg.pose.position.x
    goal.y = msg.pose.position.y

def clk_odom(msg):
    global pose
    pose.x = msg.pose.pose.position.x
    pose.y = msg.pose.pose.position.y




rospy.init_node('line_pub_example')
pub_line_min_dist = rospy.Publisher('/waypoint', Marker, queue_size=1)

sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, clk_goal)
sub_pos = rospy.Subscriber('/odometry/filtered', Odometry, clk_odom)

rospy.loginfo('Publishing example line')

while not rospy.is_shutdown():
    dist = np.hypot(goal.x - pose.x, goal.y - pose.y)
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    if(dist < 0.3):
        marker.color.r = 0.0
        marker.color.g = 1.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = goal.x
    first_line_point.y = goal.y
    first_line_point.z = 0.0
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = goal.x
    second_line_point.y = goal.y
    second_line_point.z = 5.0
    marker.points.append(second_line_point)

    # Publish the Marker
    pub_line_min_dist.publish(marker)

    rospy.sleep(0.5)