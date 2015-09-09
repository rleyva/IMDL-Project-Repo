__author__ = 'Ralph F. Leyva'

from kalman_lib import *
import geometry_lib
import roslib;
roslib.load_manifest('imu_data')
import rospy

from std_msgs.msg import Float32
from imu_data.msg import imu_data

tilt_angle = 0.00
pub_tilt = rospy.Publisher("gomboc_angle", Float32, queue_size = 20)

def imu_filtering(data):
    global tilt_angle
    gomboc_comp = kalman_filter()
    gomboc_comp.set_dt(0.1)

    #tilt_angle = gomboc_comp.compute_angle(data.accel_z, data.gyro_z)
    #tilt_angle = gomboc_comp.apply_filter(data.accel_x, data.accel_y, data.gyro_x)
    tilt_angle = gomboc_comp.com
    rospy.loginfo(tilt_angle)
    pub_tilt.publish(tilt_angle)

def imu_node():
    global pub_tilt
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("imu_data", imu_data, imu_filtering)
    rospy.spin()

if __name__ == '__main__':
    imu_node()