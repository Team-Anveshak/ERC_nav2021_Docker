#! /usr/bin/env python

# import ros stuff
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from tf import transformations
from math import pi

desired_yaw = 0
flag = 0
current_yaw = 0

def clk_angle(msg):
    global desired_yaw, current_yaw, flag
    desired_yaw = normalize_angle(current_yaw + msg.data * pi/180)
    flag = 1

def clk_odom(msg):
    global current_yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    if(msg.header.frame_id == 'odom'):
        current_yaw = euler[2]

def normalize_angle(angle):
    if(abs(angle) > pi):
        angle = angle - (2 * pi * angle) / (abs(angle))
    return angle

def main():
    global desired_yaw, current_yaw, flag

    rospy.init_node('manual_angle')

    sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, clk_odom)
    sub_imu = rospy.Subscriber('/change_angle', Float32, clk_angle)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    err_yaw = 0
    prev_err_yaw = 0
    deltaT = 10
    P = 1
    D = -10
    twist_msg = Twist()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        err_yaw = normalize_angle(desired_yaw - current_yaw)

        if(abs(err_yaw) >  0.03 and flag == 1):
            derivative = (err_yaw - prev_err_yaw)*D/deltaT
            prev_err_yaw = err_yaw
            twist_msg.angular.z = (err_yaw)*P - derivative
            pub.publish(twist_msg)
            rospy.loginfo("Desired, Current yaw: %s, %s" % (desired_yaw, current_yaw))
        elif(flag == 1):
            flag = 0
            twist_msg.angular.z = 0
            pub.publish(twist_msg)
            rospy.loginfo("Reached")
            
        rate.sleep()

if __name__ == "__main__":
	main()