import matplotlib
matplotlib.use('TkAgg')
from filterpy.kalman import UnscentedKalmanFilter as UKF
from math import *
from filterpy.kalman.sigma_points import *
import turtle
import random
from robot import *

# using 3 variables here for x (state) - [x1, y1, heading]
# this works until measurement noise starts going greater than 0.2 * target.distance
# approximately after noise exceeds 0.2, it fails to converge


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


target = robot(0.0, 15.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2 * target.distance
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -5.0, 0.0)


def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


# x are sigma points
def fx(sigmas, dt, distance, turning):
    heading = sigmas[2] + turning
    x1 = sigmas[0] + distance * cos(heading)
    y1 = sigmas[1] + distance * sin(heading)
    state = [x1, y1, heading]
    return state


def Hx(sigmas):
    return sigmas[0], sigmas[1]


def residual_h(a, b):
    y = a - b
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

# sigmas here has two columns - one for x and one for y
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
turning = 2 * np.pi / 30

sigma_range= 0.3
sigma_bearing=0.1

target_measurement = target.sense()


points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
ukf = UKF(dim_x = 3, dim_z = 2, fx=fx, hx=Hx,
          dt=dt, points=points, x_mean_fn=state_mean,
          z_mean_fn=z_mean, residual_x=residual_x,
          residual_z=residual_h)

ukf.x = np.array([target_measurement[0], target_measurement[1], 0.0]) # actual
ukf.P = np.diag([.9, .9, .9])
# ukf.R = np.diag( [sigma_range**2, sigma_bearing**2] )
ukf.R = np.diag( [5., 5.] )
ukf.Q = np.eye(3) * 0.001  # Q must not be zeroes!!! .001 is the best for this case

state = ukf.x.copy()
size_multiplier = 20

for i in range(300):

    ukf.predict(dt = 1., fx_args = (1.5, turning))
    z = target_measurement[0], target_measurement[1]
    ukf.update(z)

    # print "after update", ukf.x[0], ukf.x[1], "actual", state

    # measured_robot.goto(target_measurement[0] * size_multiplier, target_measurement[1] * size_multiplier - 200)
    # measured_robot.stamp()

    target_robot.goto(target.x * size_multiplier, target.y * size_multiplier - 200)
    target_robot.stamp()

    predicted_robot.goto(ukf.x[0] * size_multiplier, ukf.x[1] * size_multiplier - 200)
    predicted_robot.stamp()

    target.move_in_circle()
    target_measurement = target.sense()


turtle.getscreen()._root.mainloop()