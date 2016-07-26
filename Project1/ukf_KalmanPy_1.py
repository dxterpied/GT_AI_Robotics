import matplotlib
matplotlib.use('TkAgg')
from filterpy.kalman import UnscentedKalmanFilter as UKF
from math import *
from filterpy.kalman.sigma_points import *
import turtle
import random

# THIS  WORKS! it's based off of ukf_KalmanPy.py ---------------------------------------

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
measured_robot = turtle.Turtle()
measured_robot.shape('circle')
measured_robot.color('red')
measured_robot.shapesize(0.2, 0.2, 0.2)


def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


def move(x, dt, turning):
    heading = x[2] + turning
    x1 = x[0] + dt * cos(heading)
    y1 = x[1] + dt * sin(heading)
    state = [x1, y1, heading]
    return state


def fx(x, dt):
    #return move(x, u, dt, wheelbase)
    return move(x, dt, turning)


def Hx(x):
    return sense(x)


def sense(x):
    return (random.gauss(x[0], 1.5), random.gauss(x[1], 1.5))


def residual_h(a, b):
    y = a - b
    # data in format [dist_1, bearing_1, dist_2, bearing_2,...]
    for i in range(0, len(y), 2):
        y[i + 1] = normalize_angle(y[i + 1])
    return y


def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y



def state_mean(sigmas, Wm):
    x = np.zeros(3)

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = atan2(sum_sin, sum_cos)
    return x


def z_mean(sigmas, Wm):
    z_count = sigmas.shape[1]
    x = np.zeros(z_count)

    for z in range(0, z_count, 2):
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, z+1]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, z+1]), Wm))

        x[z] = np.sum(np.dot(sigmas[:,z], Wm))
        x[z+1] = atan2(sum_sin, sum_cos)
    return x


dt = 1.0
turning = -2 * np.pi / 30
landmarks = np.array([[5, 10]])
cmds = [np.array([1.1, .01])] * 30
sigma_vel=0.1
sigma_steer= np.radians(1)
sigma_range= 0.3
sigma_bearing=0.1

points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)

ukf = UKF(dim_x = 3, dim_z = 2 * len(landmarks), fx=fx, hx=Hx,
          dt=dt, points=points, x_mean_fn=state_mean,
          z_mean_fn=z_mean, residual_x=residual_x,
          residual_z=residual_h)

ukf.x = np.array([0., 0., 0.])
ukf.P = np.diag([.1, .1, .05])
ukf.R = np.diag( [sigma_range**2, sigma_bearing**2] * len(landmarks) )
ukf.Q = np.eye(3) * 0.0001

state = ukf.x.copy()

for i, u in enumerate(cmds):
    state = move(state, dt, turning)
    ukf.predict()
    z = sense(state)
    ukf.update(z)

    # measured_robot.goto(z[0] * 25, z[1] * 25)
    # measured_robot.stamp()

    # target_robot.goto(state[0] * 25, state[1] * 25)
    # target_robot.stamp()

    # print "z", z
    # print "state", state[0], state[1], "ukf x,y", ukf.x[0], ukf.x[1]

    predicted_robot.goto(ukf.x[0] * 25, ukf.x[1] * 25)
    predicted_robot.stamp()


turtle.getscreen()._root.mainloop()