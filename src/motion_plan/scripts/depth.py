#! /usr/bin/env python

# import ros stuff
import rospy
import ros_numpy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from std_srvs.srv import *
from ar_track_alvar_msgs.msg import AlvarMarkers
import pylab
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.interpolate import griddata
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


import math

xyz_array = None
pc = None

# callbacks
def clbk_camera(msg):
    global xyz_array, pc
    temp = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    # xyz_array = np.zeros(np.shape(temp))
    # xyz_array[:, 0] = temp[:, 2]
    # xyz_array[:, 1] = temp[:, 0]
    # xyz_array[:, 2] = -temp[:, 1]
    xyz_array = temp
    pc = msg

    #xyz_array = xyz_array[xyz_array[:,1] < 0]
    
def cart2sph(x,y,z):
    XsqPlusYsq = x**2 + y**2
    r = np.sqrt(XsqPlusYsq + z**2)               # r
    elev = np.arctan2(z,np.sqrt(XsqPlusYsq))     # theta
    az = np.arctan2(y,x)*180/np.pi               # phi
    return set(az.astype(int))

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, xyz_array, pc

    rospy.init_node('depth_test')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_1 = rospy.Publisher('/depth', PointCloud2, queue_size=1)

    sub_camera = rospy.Subscriber('/zed2/point_cloud/cloud_registered', PointCloud2, clbk_camera)

    # xi = np.linspace(0.5, 3, 10000)
    # yi = np.linspace(-3, 3, 10000)

    rate = rospy.Rate(1)
    rate.sleep()
    transform = tf_buffer.lookup_transform("base_link","zed2_left_camera_optical_frame", rospy.Time())
    pc_base = None
    while not rospy.is_shutdown():
        # fig, ax1 = plt.subplots(nrows=1)
        # zi = griddata((xyz_array[:, 0], xyz_array[:, 1]), xyz_array[:, 2], (xi[None, :], yi[:, None]), method='linear')
        # cp = ax1.contour(xi, yi, zi, levels=14, linewidths=0.5, colors='k')
        # fig.colorbar(cp)
        # plt.show(block=False)

        pc_base = do_transform_cloud(pc, transform) 
        pub_1.publish(pc_base)
        # xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_base)

        # fig,ax=plt.subplots(1,1)
        # cp = ax.tricontourf(xyz_array[:, 0], -xyz_array[:, 1], xyz_array[:, 2])
        # fig.colorbar(cp) # Add a colorbar to a plot
        # plt.show()


        # fig = plt.figure()
        # ax = Axes3D(fig)
        # print(np.shape(xyz_array))
        # print(cart2sph(xyz_array[:, 0], xyz_array[:, 1], xyz_array[:, 2]))
        # ax.scatter(xyz_array[:, 0], xyz_array[:, 1], xyz_array[:, 2])
        # plt.savefig('foo.png')
        # break
        rate.sleep()

if __name__ == '__main__':
    main()
