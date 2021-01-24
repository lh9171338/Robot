#! /usr/bin/env python
import rospy
from car import Car


if __name__ == '__main__':
    # Initial node
    rospy.init_node('car', anonymous=True)
    rospy.loginfo('Start the car node')

    car = Car()
    if car.setup():
        car.spin()
