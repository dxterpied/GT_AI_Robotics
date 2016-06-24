# ----------
# Part Three
#
# Now you'll actually track down and recover the runaway Traxbot.
# In this step, your speed will be about twice as fast the runaway bot,
# which means that your bot's distance parameter will be about twice that
# of the runaway. You can move less than this parameter if you'd
# like to slow down your bot near the end of the chase.
#
# ----------
# YOUR JOB
#
# Complete the next_move function. This function will give you access to
# the position and heading of your bot (the hunter); the most recent
# measurement received from the runaway bot (the target), the max distance
# your bot can move in a given timestep, and another variable, called
# OTHER, which you can use to keep track of information.
#
# Your function will return the amount you want your bot to turn, the
# distance you want your bot to move, and the OTHER variable, with any
# information you want to keep track of.
#
# ----------
# GRADING
#
# We will make repeated calls to your next_move function. After
# each call, we will move the hunter bot according to your instructions
# and compare its position to the target bot's true position
# As soon as the hunter is within 0.01 stepsizes of the target,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.
#
# As an added challenge, try to get to the target bot as quickly as
# possible.

from robot import *
from math import *
from matrix import *
import random
import time


deltaT = 0.1
x = matrix([[0.],
            [0.],
            [0.],
            [0.]]) # initial state
u = matrix([[0.], [0.], [0.], [0.]]) # external motion

F = matrix([
        [1., 0., deltaT, 0.], # this is to update x = 1 * x + 0 * y + deltaT * x_prime + 0 * y_prime = x + deltaT*x_prime
        [0., 1., 0., deltaT], # this is to update y = 0 * x + 1 * y + 0 * x_prime + deltaT * y_prime = y + deltaT * y_prime
        [0., 0., 1., 0.], # this is to update x_prime = 0 * x + 0 * y + 1 * x_prime + 0 * y_prime = x_prime
        [0., 0., 0., 1.]  # this is to update y_prime = 0 * x + 0 * y + 0 * x_prime + 1 * y_prime = y_prime
    ]) # next state function: generalize the 2d version to 4d

H = matrix([ [1., 0., 0., 0.],
            [0., 1., 0., 0.]]) # measurement function: reflect the fact that we observe x and y but not the two velocities
R = matrix([
    [0.1, 0.],
    [0., 0.1]]) # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
I = matrix([ [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]  ]) # 4d identity matrix


def next_move_KF(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    #time.sleep(0.5)
    xy_estimate = None

    if OTHER is None:
        P_matrix = matrix([
            [0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 1000., 0.],
            [0., 0., 0., 1000.] ]) # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
        measurements = [(0.0, 0.0)]
        prevAngle = 0.
    else:
        measurements, P_matrix, prevAngle = OTHER

    #print "faulty target", target_measurement
    x = matrix([[target_measurement[0]], [target_measurement[1]], [0.], [0.]])
    new_x, P_matrix = kalman_filter(x, P_matrix, measurements[-1:]) # take last n items in measurements
    adjustedTarget = (new_x.value[0][0], new_x.value[1][0]) # get new x and y from new_x matrix
    #print "adjusted target", adjustedTarget

    prevCoord = measurements[len(measurements) - 1]
    x1Delta = adjustedTarget[0] - prevCoord[0]
    y1Delta = adjustedTarget[1] - prevCoord[1]
    heading = atan2(y1Delta, x1Delta)
    turning = heading - prevAngle
    distance = distance_between(prevCoord, adjustedTarget)

    newR = robot(adjustedTarget[0], adjustedTarget[1], heading, turning, distance)
    newR.move_in_circle()
    xy_estimate = newR.x, newR.y

    prevAngle = heading
    measurements.append(adjustedTarget)
    OTHER = (measurements, P_matrix, prevAngle)


    heading_to_target = get_heading(hunter_position, xy_estimate)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)

    return turning, distance, OTHER


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""

    #time.sleep(0.5)
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
    else:
        distances, angles, coords = OTHER

        if len(coords) == 1:
            x1, y1 = coords[0]
            x2, y2 = target_measurement
            hypotenuse1 = distance_between(coords[0], target_measurement)
            y1Delta = y2 - y1
            headingAngle1 = asin(y1Delta / hypotenuse1)
            #angles.append(headingAngle1)
            distances.append(hypotenuse1)

        elif len(coords) >= 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = target_measurement

            y1Delta = point2[1] - point1[1]
            hypotenuse1 = distance_between(point1, point2)
            headingAngleAvg1 = asin(y1Delta / hypotenuse1)

            y2Delta = point3[1] - point2[1]
            x2Delta = point3[0] - point2[0]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = atan2(y2Delta, x2Delta)
            headingAngleAvg2 = asin(y2Delta / hypotenuse2)
            predictedTurnAngleAvg = headingAngleAvg2 - headingAngleAvg1

            angles.append(abs(predictedTurnAngleAvg))
            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            newR = robot(point3[0], point3[1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y

    coords.append(target_measurement)
    OTHER = (distances, angles, coords)

    if xy_estimate is None:
        xy_estimate = target_measurement

    heading_to_target = get_heading(hunter_position, xy_estimate)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)

    return turning, distance, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            return ctr
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught


def demo_grading_visual(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
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
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    #prediction.penup()
    broken_robot.penup()
    #End of Visualization

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
        prediction.stamp()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."



    return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def kalman_filter(x, P, measurements):
    for n in range(len(measurements)):

        # PREDICTION  (based on theory). Uses total probability and convolution
        x = (F * x) + u              # in Michel van Biezen it's x1 = F * x0 + B * u1 + w1: https://www.youtube.com/watch?v=mRf-cL2mjo4
        P = F * P * F.transpose() # + Q  the Q matrix (process noise) is not present here

        # MEASUREMENT UPDATE
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)  # Innovation or measurement residual
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse() # Kalman gain
        x = x + (K * y)
        P = (I - (K * H)) * P


    return x,P



target = robot(0.0, 0.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -20.0, 0.0)

demo_grading(hunter, target, next_move_KF)
#demo_grading_visual(hunter, target, next_move_KF)

# scores = []
# for i in range(10000):
#     hunter = robot(-10.0, -20.0, 0.0)
#     scores.append(demo_grading(hunter, target, next_move_KF))
# print "average score: ", sum(scores)/len(scores)
# print "minimum score: ", min(scores)
# print "maximum score: ", max(scores)
#
# stats:
# average score:  48
# minimum score:  8
# maximum score:  374

# stats with KF:
#
# average score:  169
# minimum score:  False
# maximum score:  998




