__author__ = 'Ralph F. Leyva'

# This library will implement a complementary filter
# which will be used to sensor-merge the accelerometer
# and gyroscope measurements

import geometry_lib

class complementary_filter:
    def __init__(self):
        self.angle = 0
        self.roll = 0
        self.pitch = 0
        self.a_coeff = 0
        self.g_coeff = 0
        self.sampling_time = 0.057

    # Set-ter functions
    def set_angle(self, angle):
        self.angle = angle
    def set_a_coeff(self, a_coeff):
        self.a_coeff = a_coeff
    def set_g_coeff(self, g_coeff):
        self.g_coeff = g_coeff
    def set_pitch(self, pitch):
        self.pitch = pitch
    def set_roll(self, roll):
        self.roll = roll
    def set_sampling_time(self, sampling_time):
        self.sampling_time = sampling_time

    # Get-ter functions
    def fetch_a_coeff(self):
        return self.a_coeff
    def fetch_g_coeff(self):
        return self.g_coeff
    def fetch_sampling_time(self):
        return self.sampling_time

    # Filter functions
    def apply_filter(self, accel_data, relation_plane, gyro_data):
        # Previously I had used temp_angle = geometry_lib.return_angle(accel_data, relation_plane)
        #temp_angle = geometry_lib.return_small_angle_approximation(accel_data)
        temp_angle = geometry_lib.return_angle(accel_data, relation_plane)      # Returns an angle +/-90 degrees
        self.angle = self.a_coeff*(self.angle + gyro_data * self.sampling_time) + self.g_coeff * temp_angle
        return self.angle
