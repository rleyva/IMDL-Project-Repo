__author__ = 'Ralph F. Leyva'

# This library will implement a kalman filter
# which will be used to sensor-merge the accelerometer
# and gyroscope measurements

import geometry_lib
import rospy

class kalman_filter:
    def __init__(self):
        # Default values
        self.q_angle = 0.001
        self.q_bias = 0.003
        self.r_measure = 0.03

        self.angle = 0.0
        self.bias = 0.0

        self.P00 = 0.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 0.0

        self.kalman_gain = [0,0]

        self.dt = 0.1

    def compute_angle(self, new_angle, new_rate):
        self.rate = new_rate - self.bias
        self.angle += self.dt * self.rate

        self.P00 += self.dt * (self.dt * self.P11 - self.P01 - self.P10 + self.q_angle)
        self.P01 -= self.dt * self.P11
        self.P10 -= self.dt * self.P11
        self.P11 += self.q_bias * self.dt

        S = self.P00 + self.r_measure

        self.kalman_gain[0] = self.P00 / S
        self.kalman_gain[1] = self.P10 / S

        y = new_angle - self.angle
        self.angle += self.kalman_gain[0] * y
        self.bias  += self.kalman_gain[1] * y

        P00_temp = self.P00
        P01_temp = self.P01

        self.P00 -= self.kalman_gain[0] * P00_temp
        self.P01 -= self.kalman_gain[0] * P01_temp
        self.P10 -= self.kalman_gain[1] * P00_temp
        self.P11 -= self.kalman_gain[1] * P01_temp

        return self.angle







