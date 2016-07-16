# this shit does not work................. Can't use KF with sin/cos

import matplotlib
matplotlib.use('TkAgg')
from math import *
import random
import numpy
import turtle


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


def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

# Implements a linear Kalman filter.
class KalmanFilterLinear:
  def __init__(self,_F, _H, _x, _P, _R):
    self.F = _F     # State transition matrix.
    self.H = _H     # Observation matrix.
    self.x = _x     # Initial state estimate.
    self.P = _P     # Initial covariance estimate.
    self.R = _R     # Estimated error in measurements.

  def GetCurrentState(self):
    return self.x

  def Step(self, Z, _F, B, u):
    self.F = _F
    #--- Prediction ---
    x = (self.F * self.x) + B * u
    P = (self.F * self.P) * numpy.transpose(self.F)

    #--- Observation ----
    S = self.H * P * numpy.transpose(self.H) + self.R # innovation covariance

    #---- Update ------
    K = P * numpy.transpose(self.H) * numpy.linalg.inv(S) # S inverse is like reciprocal or 1/S
    self.x = x + K * (Z - self.H * x)
    I = numpy.eye(self.P.shape[0]) # We need the size of the matrix so we can make an identity matrix.
    self.P = (I - K * self.H) * P
    return self.x

class Cannon:
  def __init__(self, _noiselevel, _distance):
    self.noiselevel = _noiselevel
    self.heading = 0.0
    self.turning = 2*pi/30
    self.distance = _distance
    self.x = -200
    self.y = -200


  def GetXWithNoise(self):
    return random.gauss( self.x, self.noiselevel)

  def GetYWithNoise(self):
    return random.gauss( self.y, self.noiselevel)

  # Increment through the next timeslice of the simulation.
  def Step(self):
    self.heading += self.turning # update the angle to create a new angle
    self.heading = angle_trunc(self.heading)
    self.x += self.distance * cos(self.heading)
    self.y += self.distance * sin(self.heading)


class robot:

    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        """This function is called when you create a new robot. It sets some of
        the attributes of the robot, either to their default values or to the values
        specified when it is created."""
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


    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):
        """This function turns the robot and then moves it forward."""
        # apply noise, this doesn't change anything if turning_noise
        # and distance_noise are zero.

        turning = random.gauss(turning, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)
        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)

        # Execute motion
        self.heading += turning # update the angle to create a new angle
        self.heading = angle_trunc(self.heading)
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)

    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        self.move(self.turning, self.distance)

    # def move_in_circle(self):
    #     """This function is used to advance the runaway target bot."""
    #     return self.move(self.turning, self.distance)


    def sense(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to,
        but not necessarily equal to, the robot's (x, y) position."""
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)

#=============================REAL PROGRAM START================================

distance = 30.
# After state transition and control, here are the equations:
#  x(n+1) = x(n) + distance * cos(self.heading)
#  y(n+1) = y(n) + distance * sin(self.heading)

# transition matrix
F = numpy.matrix([
    [1,0],
    [0,1]])
# Observation matrix is the identity matrix, since we can get direct measurements of all values in our example.
H = numpy.matrix([[1., 0.], [0. , 1.]])
x = numpy.matrix([[0.], [0.]]) # initial state vector
P = numpy.matrix([[1000., 0    ],
                  [0,     1000.]])
R = numpy.matrix([[1., 0.], [0. , 1.]]) # measurement noise
B = numpy.matrix([[1., 0.], [0. , 1.]]) # control matrix


kf = KalmanFilterLinear(F, H, x, P, R)

# Let's make a cannon simulation.
c = Cannon(_noiselevel = 0.075, _distance = distance)

target = robot(0.0, 0.0, 0.0, 2*pi / 30, 30)
measurement_noise = 1. * target.distance
target.set_noise(0.0, 0.0, measurement_noise)

# here I use exact distance and heading. In reality, those may not be known and will have to be estimated
for i in range(100):
    # Iterate the cannon simulation to the next timeslice.
    target.move_in_circle()
    target_measurement = target.sense()

    Z = numpy.matrix([[target_measurement[0]], [target_measurement[1]]])
    u = numpy.matrix([[target.distance * cos(target.heading), 0.                         ],
                      [0.,                         target.distance * sin(target.heading) ]]) # control matrix

    newState = kf.Step(Z, F, B, u)

    target_robot.goto(target.x, target.y)
    target_robot.stamp()
    predicted_robot.goto( newState.item(0), newState.item(3))
    predicted_robot.stamp()



# Iterate through the simulation.
# for i in range(100):
#     # Iterate the cannon simulation to the next timeslice.
#     c.Step()
#
#     newestX = c.GetXWithNoise()
#     newestY = c.GetYWithNoise()
#
#     Z = numpy.matrix([[newestX], [newestY]])
#     u = numpy.matrix([[c.distance * cos(c.heading), 0.                         ],
#                       [0.,                         c.distance * sin(c.heading) ]]) # control matrix
#
#     newState = kf.Step(Z, F, B, u)
#
#     target_robot.goto(c.x, c.y)
#     target_robot.stamp()
#     predicted_robot.goto( newState.item(0), newState.item(3))
#     predicted_robot.stamp()


turtle.getscreen()._root.mainloop()