__author__ = 'Ralph F. Leyva'

import roslib
import rospy
from std_msgs.msg import Float32

pub_left_motor = rospy.Publisher("left_motor_control", Float32, queue_size = 20)
pub_right_motor = rospy.Publisher("right_motor_control", Float32, queue_size = 20)

def motor_drive(data):
    # This will have to be fleshed out later when we're
    # managing individual motor stuff
    pub_right_motor.publish(data)
    pub_left_motor.publish(data)

def pid_relay():
    global pub_left_motor
    global pub_right_motor
    rospy.init_node('motor_node', anonymous=True)
    rospy.Subscriber("pid_output", Float32, motor_drive)
    rospy.spin()

if __name__ == '__main__':
    pid_relay()