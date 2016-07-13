# kalman2.py
# written by Greg Czerniak (email is greg {aT] czerniak [dOt} info )
#
# Implements a multi-variable linear Kalman filter.
#
# Note: This code is part of a larger tutorial "Kalman Filters for Undergrads"
# located at http://greg.czerniak.info/node/5.

import pylab
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
  def __init__(self,_F, _B, _H, _x, _P, _R):
    self.F = _F     # State transition matrix.
    self.B = _B     # Control matrix.
    self.H = _H     # Observation matrix.
    self.x = _x     # Initial state estimate.
    self.P = _P     # Initial covariance estimate.
    self.R = _R     # Estimated error in measurements.

  def GetCurrentState(self):
    return self.x

  def Step(self, u, Z):
    #--- Prediction -----------------------------
    x = (self.F * self.x) + (self.B * u)
    P = (self.F * self.P) * numpy.transpose(self.F)

    #--- Observation -----------------------------
    y = Z - self.H * x
    S = self.H * P * numpy.transpose(self.H) + self.R # innovation covariance

    #---- Update -------------------------------
    K = P * numpy.transpose(self.H) * numpy.linalg.inv(S)
    self.x = x + (K * y)
    I = numpy.eye(self.P.shape[0]) # We need the size of the matrix so we can make an identity matrix.
    self.P = (I - K * self.H) * P

# Simulates the classic physics problem of a cannon shooting a ball in a
# parabolic arc.  In addition to giving "true" values back, you can also ask
# for noisy values back to test Kalman filters.
class Cannon:
  #--------------------------------VARIABLES----------------------------------
  angle = 2*pi/6 # The angle from the ground to point the cannon.
  muzzle_velocity = 100 # Muzzle velocity of the cannon.
  gravity = [0, -9.81] # A vector containing gravitational acceleration.
  # The initial velocity of the cannonball
  velocity = [muzzle_velocity * cos(angle), muzzle_velocity * sin(angle)]
  loc = [-200,-200] # The initial location of the cannonball.
  acceleration = [0,0] # The initial acceleration of the cannonball.
  #---------------------------------METHODS-----------------------------------
  def __init__(self,_timeslice,_noiselevel):
    self.timeslice = _timeslice
    self.noiselevel = _noiselevel
    self.heading = 0.0
    self.turning = 2*pi/30
    self.distance = 10
    self.x = -200
    self.y = -200

  def add(self,x,y):
    return x + y

  def mult(self,x,y):
    return x * y

  def GetX(self):
    return self.x

  def GetY(self):
    return self.y

  def GetXWithNoise(self):
    return random.gauss(self.GetX(),self.noiselevel)

  def GetYWithNoise(self):
    return random.gauss(self.GetY(),self.noiselevel)

  def GetXVelocity(self):
    return self.velocity[0]

  def GetYVelocity(self):
    return self.velocity[1]

  # Increment through the next timeslice of the simulation.
  def Step(self):

    # We're gonna use this vector to timeslice everything.
    #timeslicevec = [self.timeslice,  self.timeslice]
    # Break gravitational force into a smaller time slice.
    #sliced_gravity = map(self.mult, self.gravity, timeslicevec)
    # The only force on the cannonball is gravity.
    #sliced_acceleration = sliced_gravity
    # Apply the acceleration to velocity.
    # self.velocity = map(self.add, self.velocity, sliced_acceleration)
    # sliced_velocity = map(self.mult, self.velocity, timeslicevec )
    # Apply the velocity to location.
    # self.loc = map(self.add, self.loc, sliced_velocity)

    self.heading += self.turning # update the angle to create a new angle
    self.heading = angle_trunc(self.heading)

    #self.loc = map(self.add, self.loc, [30. * cos(self.heading), 30. * sin(self.heading)])

    #self.x += self.distance * cos(self.heading)
    self.x += self.distance
    self.y += self.distance * sin(self.heading)

    # # Cannonballs shouldn't go into the ground.
    # if self.loc[1] < 0:
    #   self.loc[1] = 0



#=============================REAL PROGRAM START================================
# Let's go over the physics behind the cannon shot, just to make sure it's
# correct:
# sin(45)*100 = 70.710 and cos(45)*100 = 70.710
# vf = vo + at
# 0 = 70.710 + (-9.81)t
# t = 70.710/9.81 = 7.208 seconds for half
# 14.416 seconds for full journey
# distance = 70.710 m/s * 14.416 sec = 1019.36796 m

muzzle_velocity = 100 # How fast should the cannonball come out?
deltaT = 1
angle = 90 # Angle from the ground.
speedX = muzzle_velocity * cos(angle * pi/180)
speedY = muzzle_velocity * sin(angle * pi/180)

# This is the state transition vector, which represents part of the kinematics.
# 1, ts, 0,  0  =>  x(n+1) = x(n) + vs
# 0,  1, 0,  0  => vx(n+1) =        vx(n)
# 0,  0, 1, ts  =>  y(n+1) =              y(n) + vs * sin(self.heading)
# 0,  0, 0,  1  => vy(n+1) =                     vy(n)
# Remember, acceleration gets added to these at the control vector.
F = numpy.matrix([
    [1,1,0,0],
    [0,1,0,0],
    [0,0,1,1],
    [0,0,0,1]])

B = numpy.matrix([[0,0,0,0],
                  [0,0,0,0],
                  [0,0,1,0],
                  [0,0,0,0]])

# The control vector, which adds acceleration to the kinematic equations.
# control_vector = numpy.matrix( [[0],
#                                [0],
#                                [0.5 * -9.81 * timeslice * timeslice],
#                                [-9.81*timeslice]] )

# After state transition and control, here are the equations:
#  x(n+1) = x(n) + vx(n)
# vx(n+1) = vx(n)
#  y(n+1) = y(n) + vy(n) * sin(self.heading)
# vy(n+1) = vy(n)

# Observation matrix is the identity matrix, since we can get direct
# measurements of all values in our example.
H = numpy.eye(4)
# This is our guess of the initial state.  I intentionally set the Y value
# wrong to illustrate how fast the Kalman filter will pick up on that.
x = numpy.matrix([[0], [0] , [100], [0] ])
P = numpy.eye(4)
R = numpy.eye(4)*0.2

kf = KalmanFilterLinear(F, B, H, x, P, R)
# Let's make a cannon simulation.
c = Cannon(_timeslice = deltaT, _noiselevel = 30)

# Iterate through the simulation.
for i in range(100):
    target_robot.goto(c.GetX(), c.GetY())
    target_robot.stamp()

    newestX = c.GetXWithNoise()
    newestY = c.GetYWithNoise()
    Z = numpy.matrix([[newestX], [c.GetXVelocity()], [newestY], [c.GetYVelocity()]])

    # Iterate the cannon simulation to the next timeslice.
    c.Step()

    u = numpy.matrix( [[0],
                        [0],
                        [c.distance + 10. * cos(c.heading)],
                        [0]] )

    kf.Step(u, Z)

    predicted_robot.goto(kf.GetCurrentState()[0,0] , kf.GetCurrentState()[2,0])
    predicted_robot.stamp()


turtle.getscreen()._root.mainloop()