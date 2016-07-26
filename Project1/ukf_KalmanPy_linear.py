import matplotlib
matplotlib.use('TkAgg')

from filterpy.stats import plot_covariance_ellipse
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from math import tan, sin, cos, sqrt, atan2, radians
import matplotlib.pyplot as plt
from numpy import *
import numpy as np
from numpy.random import *
import turtle
import random
from filterpy.common import Q_discrete_white_noise


dt = 1.0

def f_cv(x, dt):
    """ state transition function for a
    constant velocity aircraft"""

    F = np.array([[1, dt, 0,  0],
                  [0,  1, 0,  0],
                  [0,  0, 1, dt],
                  [0,  0, 0,  1]])
    # print np.dot(F, x) # [ 0.  0.  0.  0.]
    return np.dot(F, x)

def h_cv(x):
    result = np.array([x[0], x[2]])
    #print result # [ 0.  0.]
    return result


points = MerweScaledSigmaPoints(n=4, alpha=.1, beta=2., kappa=-1)

exit()
ukf = UKF(dim_x=4, dim_z=2, fx=f_cv, hx=h_cv, dt=dt, points=points)

ukf.x = np.array([0., 0., 0., 0.])
ukf.R = np.diag([0.09, 0.09])
ukf.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=1, var=0.02)
ukf.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=1, var=0.02)

uxs = []

std_x, std_y = .3, .3
zs = [np.array([i + randn() * std_x, i + randn() * std_y]) for i in range(100)]

for z in zs:
    ukf.predict()
    ukf.update(z)
    uxs.append(ukf.x.copy())
uxs = np.array(uxs)

plt.plot(uxs[:, 0], uxs[:, 2])
plt.show()
