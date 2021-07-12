#! /usr/bin/env python

# import ros stuff
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist, Point, Vector3, Vector3Stamped, TwistWithCovarianceStamped
from tf import transformations, TransformListener

#Parameters
a = 0.7     #imu
b = 0.3     #odom

yaw_imu = 0
yaw_odom = 0
angular_odom = 0
yaw_weighted = 0
speed = 0
position = Point()

angular_velocity_imu = Vector3Stamped()
linear_acceleration_imu = None
linear_velocity_imu = Vector3Stamped()
tl = None

twist_cov = TwistWithCovarianceStamped()


def clk_imu(msg):
    global yaw_imu, angular_velocity_imu, tl, linear_acceleration_imu
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)

    angular_velocity_imu.header = msg.header
    angular_velocity_imu.vector.x = msg.angular_velocity.x
    angular_velocity_imu.vector.y = msg.angular_velocity.y
    angular_velocity_imu.vector.z = msg.angular_velocity.z
    angular_velocity_imu = tl.transformVector3('base_link', angular_velocity_imu)

    linear_acceleration_imu = msg.linear_acceleration
    linear_velocity_imu.header = msg.header

    yaw_imu = euler[2]

def clk_odom(msg):
    global angular_odom, speed, twist_cov
    angular_odom = msg.twist.angular.z
    speed = msg.twist.linear.x

    twist_cov.header = msg.header
    twist_cov.twist.twist = msg.twist


def main():
    global yaw_imu, yaw_odom, angular_odom, speed, position, yaw_weighted, a, b, angular_velocity_imu, tl
    global linear_acceleration_imu, linear_velocity_imu, twist_cov

    rospy.init_node('position')

    tl = TransformListener()

    sub_imu = rospy.Subscriber('/zed2/imu/data', Imu, clk_imu)
    sub_odom = rospy.Subscriber('/wheel_odom', TwistStamped, clk_odom)

    pub = rospy.Publisher('/position', Point, queue_size=1)
    pub_cov = rospy.Publisher('/wheel_odom_cov', TwistWithCovarianceStamped, queue_size=1)
    
    twist_cov.twist.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rate = rospy.Rate(10)
    rate.sleep()
    while not rospy.is_shutdown():
        yaw_odom += angular_odom * 0.1
        yaw_weighted = a*yaw_imu + b*yaw_odom

        position.x += speed*0.1*np.cos(yaw_weighted)
        position.y += speed*0.1*np.sin(yaw_weighted)

        pub.publish(position)
        pub_cov.publish(twist_cov)

        # print '[imu, odom, weighted]: (%s, %s, %s)' % (yaw_imu, yaw_odom, yaw_weighted)
        # print '%s' % angular_velocity_imu

        linear_velocity_imu.vector.x += linear_acceleration_imu.x*0.1
        linear_velocity_imu.vector.y += linear_acceleration_imu.y*0.1
        linear_velocity_imu.vector.z += linear_acceleration_imu.z*0.1
        linear_velocity_imu = tl.transformVector3('base_link', linear_velocity_imu)

        #print '%s' % linear_velocity_imu

        rate.sleep()


if __name__ == '__main__':
    main()