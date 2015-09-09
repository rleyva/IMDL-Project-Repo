__author__ = 'Ralph F. Leyva'

from pid_lib import *

import roslib
import rospy
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg

# "Global" variables
gomboc_pid = pid_controller()
pid_sample_time = 0.025
# If you want to demo, set kp to 0, and everything to 0
pid_kp = 300
pid_ki = 0
pid_kd = 0
pid_output = 0
pub_pid = rospy.Publisher("pid_output", Float32, queue_size = 10)

def pid_init():
    global gomboc_pid
    global rate
    gomboc_pid = pid_controller()
    gomboc_pid.set_pid_mode(True)
    gomboc_pid.set_output_limits(-80, 80)
    gomboc_pid.set_sample_time(pid_sample_time)
    rate = rospy.Rate(1/gomboc_pid.sample_time)
    gomboc_pid.set_tunings(pid_kp, pid_ki, pid_kd, pid_sample_time)
    gomboc_pid.set_set_point(0)

def pid_refresh(data):
    global pid_output
    global rate
    pid_output = gomboc_pid.pid_test_compute(data)
    rospy.loginfo(pid_output)
    pub_pid.publish(pid_output)
    rate.sleep()

def input_listener():
    rospy.Subscriber("gomboc_angle", Float32, pid_refresh)
    rospy.spin()

if __name__ == '__main__':
    global rate
    rospy.init_node("pid_plant", anonymous=True)
    pid_init()
    input_listener()