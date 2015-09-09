#!/usr/bin/env python
__author__ = 'Ralph F. Leyva'

import roslib
roslib.load_manifest('distance_data')
import rospy
import ultrasonic_lib

from distance_data.msg import distance_data

def callback(data):
    rospy.loginfo("%s", data)

def distance_listener():
    rospy.init_node('distance_listener', anonymous=True)
    rospy.Subscriber("distance_data", distance_data, callback)
    rospy.spin()

if __name__ == '__main__':
    distance_listener()

