__author__ = 'Ralph F. Leyva'

class ul_dist:
    def __init__(self):
        self.left_sensor_distance = 0
        self.center_sensor_distance = 0
        self.right_sensor_distance = 0

    def fetch_left_sensor_distance(self):
        return self.left_sensor_distance

    def fetch_right_sensor_distance(self):
        return self.right_sensor_distance

    def fetch_center_sensor_distance(self):
        return self.center_sensor_distance

    def set_left_sensor(self, distance_value):
        self.left_sensor_distance = distance_value

    def set_right_sensor(self, distance_value):
        self.right_sensor_distance = distance_value

    def set_center_sensor(self, distance_value):
        self.center_sensor_distance = distance_value
