#! /usr/bin/env python
import rospy
from lidar import Lidar


if __name__ == '__main__':
    # Initial node
    rospy.init_node('lidar', anonymous=True)
    rospy.loginfo('Start the lidar node')

    lidar = Lidar()
    if lidar.setup():
        lidar.spin()
