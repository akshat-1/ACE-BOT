import math
import numpy
from params import *


def inv_kin_fn(goal_position):
    #Compute joint angles [in degrees] to reach desired position [in meters] 
    x_desired = goal_position[0]
    y_desired = goal_position[1]
    

    cos = (x_desired**2 + y_desired**2 - lengths[1]**2 - lengths[2]**2)/(2*lengths[1]*lengths[2]) 
    sin = math.sqrt(1-cos**2)

    theta_2 = numpy.arctan2(sin,cos)
    M = lengths[1] + lengths[2]*cos
    N = lengths[2]*sin
    y = numpy.arctan2(N,M)

    theta_1 = numpy.arctan2(y_desired, x_desired) - y
    ### Fill this part ###
    theta_1 = numpy.rad2deg(theta_1)
    theta_2 = numpy.rad2deg(theta_2)
    return [11*theta_1, theta_2]

print(inv_kin_fn([15,0]))