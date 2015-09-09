__author__ = 'Ralph F. Leyva'

## Script that will carry out a rudimentary obstacle
## avoidance algorithm. This node will subscribe to the
## distance_data message being published by the Arduino Mega
## and publish a corresponding message.

from ultrasonic_lib import *
import roslib
roslib.load_manifest('distance_data')
import rospy

from distance_data.msg import distance_data

def obs_avoidance(ultrasonic_sensor_data):
    # Parses incoming message into a custom type... I don't think this is necessary!
    CONST_THRESHHOLD = 50

    current_distance = ul_dist()

    current_distance.set_center_sensor(ultrasonic_sensor_data.center_ultrasonic_sensor_distance)
    current_distance.set_left_sensor(ultrasonic_sensor_data.left_ultrasonic_sensor_distance)
    current_distance.set_right_sensor(ultrasonic_sensor_data.right_ultrasonic_sensor_distance)

    # You want to have a running average that will manage
    if(current_distance.fetch_left_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() > CONST_THRESHHOLD):
        rospy.loginfo("All is well on the western plains")
    elif(current_distance.fetch_left_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() < CONST_THRESHHOLD):
        rospy.loginfo("Obstacle is approaching from the center - Turn either left or right")
    elif(current_distance.fetch_left_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() > CONST_THRESHHOLD):
        rospy.loginfo("Obstacle approaching from the right - Continue the course, or turn left")
    elif(current_distance.fetch_left_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() < CONST_THRESHHOLD):
        rospy.loginfo("Obstacle approaching from the right and center - Turn left")
    elif(current_distance.fetch_left_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() > CONST_THRESHHOLD):
        rospy.loginfo("Obstacle approaching from the left - Continue the course or turn right")
    elif(current_distance.fetch_left_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() > CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() < CONST_THRESHHOLD):
        rospy.loginfo("Obstacle approaching from the left, and center - Turn right")
    elif(current_distance.fetch_left_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() > CONST_THRESHHOLD):
        rospy.loginfo("Obstacle approaching from the left and right - Reconsider your heading")
    elif(current_distance.fetch_left_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_right_sensor_distance() < CONST_THRESHHOLD and current_distance.fetch_center_sensor_distance() < CONST_THRESHHOLD):
        rospy.loginfo("We're about to hit something! - Reverse")
    else:
        rospy.loginfo("UKNOWN CASE")

def distance_listener():
    rospy.init_node('distance_listener', anonymous=True)
    rospy.Subscriber("distance_data", distance_data, obs_avoidance)
    rospy.spin()

if __name__ == '__main__':
    distance_listener()
