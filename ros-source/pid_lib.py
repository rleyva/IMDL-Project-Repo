#!/usr/bin/env python
__author__ = 'Ralph F. Leyva'

import time
import rospy
import numpy

class pid_controller:
    def __init__(self):
        self.pid_auto_mode = True
        self.kd = 0
        self.ki = 0
        self.kp = 0

        self.set_point = 0
        self.prev_time = 0
        self.sample_time = 0

        self.error = 0
        self.error_sum = 0
        self.last_error = 0
        self.last_time = 0

        self.last_input = 0

        self.sample_time = 0.05

        self.integral_term = 0

        self.minimum_output = 0
        self.NULL_out = 0           # Corresponds to no output on the motors
        self.maximum_output = 0     # This is dependant on the hardware being used. In my case, my
                                    # motor driver can receive a value from -400 up to 400. These
                                    # values correspond to the amount of current being sent to the
                                    # motors.
        self.output = 0

    # Set-ter functions
    def set_pid_mode(self, mode):
        if self.pid_auto_mode == False:
            self.pid_auto_mode = mode
        else:
            self.pid_auto_mode = mode

    def re_init_pid(self, plant_input, prev_output):
        self.last_input = plant_input
        self.integral_term = prev_output

        if self.integral_term > self.maximum_output:
            self.integral_term = self.maximum_output
        elif self.integral_term < self.minimum_output:
            self.integral_term = self.minimum_output

    def set_kd(self, kd, sample_time):
        self.kd = kd

    def set_ki(self, ki, sample_time):
        self.ki = ki

    def set_kp(self, kp, sample_time):
        self.kp = kp

    def set_tunings(self, kp, ki, kd, sample_time):
        self.kp = kp
        self.ki = ki * sample_time
        self.kd = kd / sample_time

    def set_set_point(self, set_point):
        self.set_point = set_point

    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    def set_maximum_output(self, maximum_output):
        self.maximum_output = 400

    def set_NULL_out(self, NULL_out):
        self.NULL_out = NULL_out

    def set_minimum_output(self, minimum_output):
        self.minimum_output = minimum_output

    def set_output_limits(self, minimum_output, maximum_output):
        self.minimum_output = minimum_output
        self.maximum_output = maximum_output

    # Get-ter functions
    def return_kp(self):
        return self.kp

    def return_ki(self):
        return self.ki

    def return_kd(self):
        return self.kd

    def return_tunings(self):
        return self.kp, self.ki, self.kd

    def return_set_point(self):
        return self.set_point

    def return_error(self):
        return self.error

    def check_limits(self):
        if self.output > self.maximum_output:
            self.output = self.maximum_output
        elif self.output < self.minimum_output:
            self.output = self.minimum_output

        if self.integral_term > self.maximum_output:    # Used for cases when the PID reaches its extreme conditions. In this case the PID may be telling
            self.integral_term = self.maximum_output    # the motor control node to output a value greater than +/-400mA, but the driver cannot manage that
        elif self.integral_term < self.minimum_output:  # so the output of the PID is clamped to the maximum values that the driver can support
            self.integral_term = self.minimum_output

    # Computation functions
    def pid_compute(self, plant_input):
        #now_time = time.time()
        #time_change = (now_time - self.prev_time)
        self.error = self.set_point - plant_input.data
        self.integral_term += (self.ki * self.error)    # Used to prevent erratic behavior when the filter parameters are changed while the filter is running

       # self.error_sum += (self.error * 0.1)
        derivative_input = (plant_input.data - self.last_input)      # We use this to protect against changing setpoints which would cause the PID to react sporadically
        #derivative_error = (self.error - self.last_error)      # Used before the previous line was added
        self.output = (self.kp * self.error + self.integral_term - self.kd * derivative_input)
        self.check_limits()
        self.last_error = self.error
        self.last_input = plant_input.data
        #self.prev_time  = now_time
        return self.output

    def pid_test_compute(self, plant_input):
        self.error = self.set_point - plant_input.data
        self.error_sum = self.error * self.sample_time
        derivative_error = (self.error - self.last_error)/self.sample_time
        self.output = self.kp * self.error + self.ki * self.error_sum + self.kd * derivative_error
        self.check_limits()
        self.last_error = self.error
        return self.output
