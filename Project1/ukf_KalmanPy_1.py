import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import scipy.linalg as linalg
import matplotlib.pyplot as plt
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise
from math import *
from filterpy.stats import plot_covariance_ellipse
from filterpy.kalman.sigma_points import *
import turtle
import random

# THIS ONE WORKS! it's based off of ukf_KalmanPy.py ---------------------------------------


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


def move_old(x, u, dt, wheelbase):
    hdg = x[2]
    vel = u[0]
    steering_angle = u[1]
    dist = vel * dt

    if abs(steering_angle) > 0.001: # is robot turning?
        beta = (dist / wheelbase) * tan(steering_angle)
        r = wheelbase / tan(steering_angle) # radius

        sinh, sinhb = sin(hdg), sin(hdg + beta)
        cosh, coshb = cos(hdg), cos(hdg + beta)
        result = x + np.array([-r*sinh + r*sinhb,
                              r*cosh - r*coshb, beta])
    else: # moving in straight line
        result = x + np.array([dist*cos(hdg), dist*sin(hdg), 0])
    #print "move", result # [ 20.28326362  16.25589597   0.72241408]
    return result

def fx(x, dt, u):
    #return move(x, u, dt, wheelbase)
    return move(x, dt, turning)


def Hx(x):
    return sense(x)

def Hx_old(x, landmarks):
    """ takes a state variable and returns the measurement (distance and angle) that would correspond to that state. """
    hx = []
    for lmark in landmarks:
        px, py = lmark
        dist = sqrt((px - x[0])**2 + (py - x[1])**2)
        angle = atan2(py - x[1], px - x[0])
        hx.extend([dist, normalize_angle(angle - x[2])])
    result = np.array(hx)
    print "Hx", result # [ 16.40870679   2.81858409]
    return result

def sense(x):
    return (random.gauss(x[0], 0.075), random.gauss(x[1], 0.075))



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


wheelbase = 0.5
dt = 1.0
turning = 2 * np.pi / 30

def run_localization(
    cmds, landmarks, sigma_vel, sigma_steer, sigma_range,
    sigma_bearing, ellipse_step=1, step=10):

    plt.figure()
    points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
    ukf = UKF(dim_x=3, dim_z=2*len(landmarks), fx=fx, hx=Hx,
              dt=dt, points=points, x_mean_fn=state_mean,
              z_mean_fn=z_mean, residual_x=residual_x,
              residual_z=residual_h)

    ukf.x = np.array([2, 6, .3])
    ukf.P = np.diag([.1, .1, .05])
    ukf.R = np.diag([sigma_range**2,
                     sigma_bearing**2]*len(landmarks))
    ukf.Q = np.eye(3)*0.0001

    sim_pos = ukf.x.copy()

    # plot landmarks
    if len(landmarks) > 0:
        plt.scatter(landmarks[:, 0], landmarks[:, 1],
                    marker='s', s=60)

    track = []
    for i, u in enumerate(cmds):

        # sim_pos = move(sim_pos, u, dt/step, wheelbase)
        sim_pos = move(sim_pos, dt, turning)
        target_robot.goto(sim_pos[0] * 25, sim_pos[1] * 25)
        target_robot.stamp()

        # track.append(sim_pos)

        if i % step == 0:
            ukf.predict(fx_args=u)

            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=6,
                     facecolor='k', alpha=0.3)

            x, y = sim_pos[0], sim_pos[1]
            z = []
            z = sense(sim_pos)

            # for lmark in landmarks:
            #     dx, dy = lmark[0] - x, lmark[1] - y
            #     d = sqrt(dx**2 + dy**2) + np.random.randn() * sigma_range
            #     bearing = atan2(lmark[1] - y, lmark[0] - x)
            #     a = (normalize_angle(bearing - sim_pos[2] +
            #          np.random.randn() * sigma_bearing))
            #     z.extend([d, a])

            ukf.update(z)

            predicted_robot.goto(ukf.x[0] * 25, ukf.x[1] * 25)
            predicted_robot.stamp()

            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=6,
                     facecolor='g', alpha=0.8)

    # track = np.array(track)
    # plt.plot(track[:, 0], track[:,1], color='k', lw=2)
    # plt.axis('equal')
    # plt.title("UKF Robot localization")
    # #plt.show()
    return ukf


landmarks = np.array([[5, 10]])
cmds = [np.array([1.1, .01])] * 100

ukf = run_localization(cmds, landmarks, sigma_vel=0.1, sigma_steer=np.radians(1), sigma_range=0.3, sigma_bearing=0.1)


#print(np.degrees(normalize_angle(np.radians(1-359))))