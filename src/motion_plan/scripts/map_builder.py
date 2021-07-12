#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import numpy as np

trav_map = OccupancyGrid()

def clbk_trav(msg):
    global trav_map

    trav_map = msg


def main():
    global trav_map

    rospy.init_node('map_builder')
    sub_trav = rospy.Subscriber('/grid_map_visualization/traversability_grid', OccupancyGrid, clbk_trav)

    pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)

    rate = rospy.Rate(5)
    rate.sleep()
    cost_map = None
    while not rospy.is_shutdown():
        cost_map = trav_map
        cost_map.data = np.asarray(cost_map.data)
        cost_map.data[cost_map.data < 60] = 0
        cost_map.data[cost_map.data > 60] = 100

        temp = np.reshape(cost_map.data, (cost_map.info.height, cost_map.info.width))
        temp[:5, :] = 100
        temp[-5:, :] = 100
        temp[:, :5] = 100
        temp[:, -5:] = 100

        pub.publish(cost_map)
        print np.shape(temp)

        rate.sleep()



if __name__ == '__main__':
    main()