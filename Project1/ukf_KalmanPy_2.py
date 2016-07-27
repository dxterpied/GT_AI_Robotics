from filterpy.kalman import UnscentedKalmanFilter as UKF
from math import *
from filterpy.kalman.sigma_points import *
import turtle
import random
from robot import *


# attempts to solve RR2 using UKF from KalmanPy
# works badly; needs further work; based on ukf_KalmanPy_1.py ---------------------------------------

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

test_target = robot(0., 0., 0., 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

sigma_vel=0.1
sigma_steer= np.radians(1)
sigma_range= 0.3
sigma_bearing= 0.1


def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


def fx(x, dt, turning):
    prevsiousHeading = x[2]
    heading = prevsiousHeading + turning
    x1 = x[0] + dt * cos(heading)
    y1 = x[1] + dt * sin(heading)
    state = [x1, y1, heading]
    return state


def Hx(x):
    result = test_target.sense()

    #result = result[0], result[1], 1.

    return result


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


def z_mean(sigmas, Wm):
    z_count = sigmas.shape[1]
    x = np.zeros(z_count)

    for z in range(0, z_count, 2):
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, z+1]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, z+1]), Wm))

        x[z] = np.sum(np.dot(sigmas[:,z], Wm))
        x[z+1] = atan2(sum_sin, sum_cos)
    return x


# cross product
def calculateRotationDirection(Ax, Ay, Bx, By, Cx, Cy):
    return ((Bx - Ax) * (Cy - By)) - ((By - Ay) * (Cx - Bx))


def getRotationSign(rotationAngles):
    # some will be negative; some positive; count which one has more elements
    positive = [i for i in rotationAngles if i > 0.0]
    negative = [i for i in rotationAngles if i < 0.0]

    if len(positive) > len(negative):
        return 1
    else:
        return -1



def estimate_next_pos(measurement, OTHER = None):

    xy_estimate = measurement


    if OTHER is None:
        distances = []
        angles = []
        coords = []
        turnAngle = []
        ukf = None
    else:
        distances, angles, coords, turnAngle, ukf = OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], measurement)
            distances.append(hypotenuse1)
        elif len(coords) >= 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = measurement

            turnAngle.append(calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1]))
            rotationSign = getRotationSign(turnAngle)

            y1Delta = point2[1] - point1[1]
            hypotenuse1 = distance_between(point1, point2)

            headingAngleAvg1 = asin(y1Delta / hypotenuse1)

            y2Delta = point3[1] - point2[1]
            x2Delta = point3[0] - point2[0]
            hypotenuse2 = distance_between(point2, point3)

            headingAngleAvg2 = asin(y2Delta / hypotenuse2)
            headingAngle2 = atan2(y2Delta, x2Delta)
            predictedTurnAngleAvg = headingAngleAvg2 - headingAngleAvg1

            angles.append(abs(predictedTurnAngleAvg))
            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            if ukf is None:
                points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)

                ukf = UKF(dim_x = 3, dim_z = 2, fx=fx, hx=Hx,
                          dt=1.5, points=points, x_mean_fn=state_mean,
                          z_mean_fn=z_mean, residual_x=residual_x,
                          residual_z=residual_h)
                ukf.x = np.array([measurement[0], measurement[1], headingAngle2])
                ukf.P = np.diag([.1, .1, .1])
                ukf.R = np.diag( [sigma_range**2, sigma_bearing**2] )
                ukf.Q = np.diag([0., 0., 0.])


            ukf.predict(dt = avgDT, fx_args = avgAngle)
            #ukf.predict()
            ukf.update(measurement)


            # newR = robot(ukf.x[0], ukf.x[1], headingAngle2, rotationSign * avgAngle, avgDT)
            # newR.move_in_circle()
            #xy_estimate = newR.x, newR.y
            xy_estimate = ukf.x[0], ukf.x[1]

    coords.append(measurement)
    OTHER = (distances, angles, coords, turnAngle, ukf)

    return xy_estimate, OTHER



# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    import sys
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)

        #print "target_bot.heading", target_bot.heading
        # if ctr >= 34:
        #     exit()
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            return ctr
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
            return 1000
    return localized


def demo_grading_visual(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.2, 0.2, 0.2)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    #prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
        #More Visualization
        # measured_broken_robot.setheading(target_bot.heading*180/pi)
        # measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        # measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized



#points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)

# ukf = UKF(dim_x = 3, dim_z = 2, fx=fx, hx=Hx,
#           dt=dt, points=points, x_mean_fn=state_mean,
#           z_mean_fn=z_mean, residual_x=residual_x,
#           residual_z=residual_h)
# ukf.x = np.array([0., 0., 0.])
# ukf.P = np.diag([.1, .1, .1])
# ukf.R = np.diag( [sigma_range**2, sigma_bearing**2] )
# ukf.Q = np.eye(3) * 0.0001


#demo_grading(estimate_next_pos, test_target)
demo_grading_visual(estimate_next_pos, test_target)


# scores = []
# fails = 0
# for i in range(1000):
#     print i
#     test_target = robot(0., 0., 0., 2*pi / 34.0, 1.5)
#     measurement_noise = 0.05 * test_target.distance
#     test_target.set_noise(0.0, 0.0, measurement_noise)
#
#     points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
#
#     ukf = UKF(dim_x = 3, dim_z = 2, fx=fx, hx=Hx,
#               dt=dt, points=points, x_mean_fn=state_mean,
#               z_mean_fn=z_mean, residual_x=residual_x,
#               residual_z=residual_h)
#     ukf.x = np.array([0., 0., 0.])
#     ukf.P = np.diag([.1, .1, .1])
#     ukf.R = np.diag( [sigma_range**2, sigma_bearing**2] )
#     ukf.Q = np.eye(3) * 0.0001
#
#     score = demo_grading(estimate_next_pos, test_target)
#
#     if score == 1000:
#         fails += 1
#     else:
#         scores.append(score)
#
# print "average score: ", sum(scores)/ float(len(scores))
# print "minimum score: ", min(scores)
# print "maximum score: ", max(scores)
# print "fails: ", fails

# these are the results with hardcoded dt and heading; need to make them dynamic
# average score:  18.579
# minimum score:  3
# maximum score:  101
# fails:  0


#turtle.getscreen()._root.mainloop()