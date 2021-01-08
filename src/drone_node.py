#! /usr/bin/env python
import rospy
from drone import Drone


if __name__ == '__main__':
    # Initial node
    rospy.init_node('drone', anonymous=True)
    rospy.loginfo('Start the drone node')

    drone = Drone()
    if drone.setup():
        drone.spin()
