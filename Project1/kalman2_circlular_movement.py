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
    y = Z - self.H * x
    S = self.H * P * numpy.transpose(self.H) + self.R # innovation covariance

    #---- Update ------
    K = P * numpy.transpose(self.H) * numpy.linalg.inv(S)
    self.x = x + (K * y)
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
c = Cannon(_noiselevel = 0.01, _distance = distance)

# Iterate through the simulation.
for i in range(100):
    # Iterate the cannon simulation to the next timeslice.
    c.Step()

    newestX = c.GetXWithNoise()
    newestY = c.GetYWithNoise()

    Z = numpy.matrix([[newestX], [newestY]])
    u = numpy.matrix([[c.distance * cos(c.heading), 0.                         ],
                      [0.,                         c.distance * sin(c.heading) ]]) # control matrix

    newState = kf.Step(Z, F, B, u)

    target_robot.goto(c.x, c.y)
    target_robot.stamp()
    predicted_robot.goto( newState.item(0), newState.item(1))
    predicted_robot.stamp()


turtle.getscreen()._root.mainloop()