__author__ = 'Ralph F. Leyva'

import math

# This library will implement some basic geometry for
# calculating angle vectors for use with the complementary
# filter

def return_dist(vector_a, vector_b):
    # Returns magnitude of the distance between two points
    return math.sqrt((vector_a*vector_a) + (vector_b*vector_b))

def return_angle(vector_a, vector_b):
    return ((math.atan2(vector_a, vector_b))*(180/math.pi) - 90)

def return_small_angle_approximation(vector_a):
    # There lie domain problems here!
    return math.asin(vector_a/10)*(180/math.pi)     # 10.8 is our magic number!