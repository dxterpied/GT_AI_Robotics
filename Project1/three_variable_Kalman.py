# this is my attempt to implement three variable Kalman filter

# we have three variables: x, y and velocity

from math import *
import random
import turtle


turtle.setup(800, 800)
window = turtle.Screen()
window.bgcolor('white')

size_multiplier= 25.0  #change Size of animation

target_robot = turtle.Turtle()
target_robot.shape('turtle')
target_robot.color('green')
target_robot.resizemode('user')
target_robot.shapesize(0.2, 0.2, 0.2)


broken_robot = turtle.Turtle()
broken_robot.shape('circle')
broken_robot.color('red')
broken_robot.resizemode('user')
broken_robot.shapesize(0.2, 0.2, 0.2)
broken_robot.penup()


predicted_robot = turtle.Turtle()
predicted_robot.shape('circle')
predicted_robot.color('blue')
predicted_robot.resizemode('user')
predicted_robot.shapesize(0.2, 0.2, 0.2)

class matrix:

    # implements basic operations of a matrix class

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)


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


def kalman_filter(x, P, measurement):

    # PREDICTION  (based on theory). Uses total probability and convolution
    x = (F * x)               # in Michel van Biezen it's x1 = F * x0 + B * u1 + w1: https://www.youtube.com/watch?v=mRf-cL2mjo4
    P = F * P * F.transpose() # + Q  the Q matrix (process noise) is not present here

    # MEASUREMENT UPDATE
    Z = matrix([measurement])
    y = Z.transpose() - (H * x)  # Innovation or measurement residual
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse() # Kalman gain
    x = x + (K * y)
    P = (I - (K * H)) * P

    return x,P

# state variables are in this order: x, y, x_prime, y_prime
deltaT = 1.0 # time interval; in this case it's arbitrary because we are not measuring real time

# initial state
x = matrix([[0.], # x
            [0.], # y
            [0.], # x prime
            [0.]]) # y prime
P = matrix([
            [1000., 0., 0., 0.],
            [0., 1000., 0., 0.],
            [0., 0., 1000., 0.],
            [0., 0., 0., 1000.] ]) # initial uncertainty
F = matrix([
        [1., 0., deltaT, 0.], # update x = 1 * x + 0 * y + deltaT * x_prime + 0 * y_prime = x + deltaT * x_prime
        [0., 1., 0., deltaT], # update y = 0 * x + 1 * y + 0 * x_prime + deltaT * y_prime = y + deltaT * y_prime
        [0., 0., 1., 0.],     # update x_prime = 0 * x + 0 * y + 1 * x_prime + 0 * y_prime = x_prime
        [0., 0., 0., 1.]      # update y_prime = 0 * x + 0 * y + 0 * x_prime + 1 * y_prime = y_prime
    ]) # next state function

H = matrix([ [1., 0., 0., 0.],
            [0., 1., 0., 0.]]) # measurement function: reflect the fact that we observe x and y but not the two velocities
R = matrix([[0.1, 0.],
            [0., 0.1]]) # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
I = matrix([ [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]  ]) # 4d identity matrix


target = robot(-10.0, -0.0, 0.0, 2*pi / 30, 1)
target.set_noise(0.0, 0.0, 0.2)


new_x = x
P_matrix = P

for i in range(25):

    target.moveStraight()
    measurements = target.sense()
    target_robot.goto(target.x * size_multiplier, target.y * size_multiplier)
    target_robot.stamp()

    new_x, P_matrix = kalman_filter(new_x, P_matrix, measurements)

    predictedX = new_x.value[0][0]
    predictedY = new_x.value[1][0]

    predicted_robot.goto(predictedX * size_multiplier, predictedY * size_multiplier)
    predicted_robot.stamp()

    broken_robot.goto(measurements[0] * size_multiplier, measurements[1] * size_multiplier)
    broken_robot.stamp()


turtle.getscreen()._root.mainloop()

