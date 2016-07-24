'''

This is from pykalman

==============================================
Using the Unscented Kalman Filter and Smoother
==============================================

This simple example shows how one may apply the Unscented Kalman Filter and
Unscented Kalman Smoother to some randomly generated data.

The Unscented Kalman Filter (UKF) and Rauch-Rung-Striebel type Unscented Kalman
Smoother (UKS) are a generalization of the traditional Kalman Filter and
Smoother to models with non-linear equations describing state transitions and
observation emissions. Unlike the Extended Kalman Filter (EKF), which attempts
to perform the same task by using the numerical derivative of the appropriate
equations, the UKF selects a handful of "sigma points", passes them through the
appropriate function, then finally re-estimates a normal distribution around
those propagated points. Experiments have shown that the UKF and UKS are
superior to the EKF and EKS in nearly all scenarios.

The figure drawn shows the true, hidden state; the state estimates given by the
UKF; and finally the same given by the UKS.
'''
import matplotlib
matplotlib.use('TkAgg')

import numpy as np
import pylab as pl
from UKF import *
import turtle
from math import *
import random

turtle.setup(800, 800)
window = turtle.Screen()
window.bgcolor('white')

target_robot = turtle.Turtle()
target_robot.shape('turtle')
target_robot.color('green')
target_robot.shapesize(0.2, 0.2, 0.2)

predicted_robot = turtle.Turtle()
predicted_robot.shape('circle')
predicted_robot.color('blue')
predicted_robot.shapesize(0.2, 0.2, 0.2)

size_multiplier = 25

# F matrix - next state function
def transition_function(state, noise):
    a = np.sin(state[0]) + state[1] * noise[0]
    b = state[1] + noise[1]
    return np.array([a, b])

# H matrix - measurement function
def observation_function(state, noise):
    C = np.array([[-1, 0.5], [0.2, 0.1]])
    return np.dot(C, state) + noise



transition_covariance = np.eye(2)  # identity matrix
# [[ 1.  0.]
#  [ 0.  1.]]
random_state = np.random.RandomState(0)
measurement_uncertainty = np.eye(2) + random_state.randn(2, 2) * 0.1  # R matrix;  measurement uncertainty
# [[ 1.17640523  0.04001572]
#  [ 0.0978738   1.22408932]]
initial_state_mean = [0, 0] # x matrix  # state matrix
initial_state_covariance = [[1, 0.1],
                            [-0.1, 1]]

def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:

    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0


    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
        helpful when using particle filters."""
        self.turning_noise    = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    def moveInCircle(self):
        # Execute motion
        self.heading += self.turning # update the angle to create a new angle
        self.heading = angle_trunc(self.heading)
        self.x += self.distance * cos(self.heading)
        self.y += self.distance * sin(self.heading)

    def moveStraight(self):
        self.x += self.distance
        self.y += self.distance/2.0

    def sense(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to,
        but not necessarily equal to, the robot's (x, y) position."""
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)


def generateStatesAndObservations():
    states = []
    observations = []

    target = robot(-10.0, -0.0, 0.0, 2*pi / 30, 1.5)
    target.set_noise(0.0, 0.0, 0.2)


    for i in range(30):
        target.moveInCircle()
        states.append([target.x, target.y])
        observations.append(target.sense())

    return (states, observations)


states = [[0., 0.], [2. ,2.], [5., 5.]]
observations = [[0.1, 0.2], [2.1 ,2.2], [5.0,5.1]]



# sample from model
kf = UKF(
    transition_function, observation_function,
    transition_covariance, measurement_uncertainty,
    initial_state_mean, initial_state_covariance,
    random_state=random_state
)

#states, observations = kf.sample(20, initial_state_mean)
states, observations = generateStatesAndObservations()
# estimate state with filtering and smoothing

filtered_state_estimates = kf.filter(observations)[0]

# print filtered_state_estimates
print states

for i in range(len(filtered_state_estimates)):
    target_robot.goto(states[i][0] * size_multiplier, states[i][1] * size_multiplier)
    target_robot.stamp()

    predicted_robot.goto(filtered_state_estimates[i][0] * size_multiplier,filtered_state_estimates[i][1] * size_multiplier)
    predicted_robot.stamp()



#draw estimates
pl.figure()
lines_true = pl.plot(states, color='b')
lines_filt = pl.plot(filtered_state_estimates, color='r', ls='-')
pl.legend((lines_true[0], lines_filt[0]), ('actual data', 'UKF'), loc='lower left')
pl.show()

turtle.getscreen()._root.mainloop()

