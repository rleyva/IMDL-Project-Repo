__author__ = 'Ralph F. Leyva'

from complementary_lib import *
import roslib;
roslib.load_manifest('imu_data')
import rospy

from std_msgs.msg import Float32
from imu_data.msg import imu_data

import math

tilt_angle = 0.00
pub_tilt = rospy.Publisher("gomboc_angle", Float32, queue_size = 10)

def imu_filtering(data):
    global tilt_angle
    gomboc_comp = complementary_filter()
    # Time-constant for filter is equal to 0.488 sec.
    gomboc_comp.set_a_coeff(0.02)
    gomboc_comp.set_g_coeff(0.98)
    gomboc_comp.set_sampling_time(0.057)
    tilt_angle = gomboc_comp.apply_filter(data.accel_z, data.accel_y, math.degrees(data.gyro_z))
    rospy.loginfo(tilt_angle)
    pub_tilt.publish(tilt_angle)

def imu_node():
    global pub_tilt
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("imu_data", imu_data, imu_filtering)
    rospy.spin()

if __name__ == '__main__':
    imu_node()