# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completely noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random
from numpy import *

# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.

def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    headingAngle1 = 0.0
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    xy_estimate = (3.2, 9.1)
    if OTHER is None:
        distances = []
        angles = []
        coords = []
    else:
        distances, angles, coords = OTHER

        if len(coords) == 1:
            x1, y1 = coords[0]
            x2, y2 = measurement
            hypotenuse1 = distance_between(coords[0], measurement)
            y1Delta = y2 - y1
            headingAngle1 = asin(y1Delta / hypotenuse1)
            angles.append(headingAngle1)
            distances.append(hypotenuse1)

        elif len(coords) == 2:
            point1 = coords[0]
            point2 = coords[1]
            point3 = measurement

            y1Delta = point2[1] - point1[1]
            hypotenuse1 = distance_between(point1, point2)
            headingAngle1 = asin(y1Delta / hypotenuse1)

            y2Delta = point3[1] - point2[1]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = asin(y2Delta / hypotenuse2)

            angles.append(abs(headingAngle2 - headingAngle1))
            distances.append(hypotenuse2)

        else:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = measurement

            y1Delta = point2[1] - point1[1]
            x1Delta = point2[0] - point1[0]
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

            #print "avgDT:", avgDT, "avgAngle:", avgAngle, "headingAngle2:", headingAngle2

            #print "avgAngle:", avgAngle
            newR = robot(point3[0], point3[1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y
        #print "headingAngle1", headingAngle1

    coords.append(measurement)
    OTHER = (distances, angles, coords)

    return xy_estimate, OTHER


def estimate_next_pos_2(measurement, OTHER = None):
    xy_estimate = 0, 0
    if OTHER is None:
        OTHER = []
        OTHER.append(measurement)
    elif len(OTHER) < 3:
        OTHER.append(measurement)
    elif len(OTHER) >= 3:
        OTHER.append(measurement)
        turning = 0
        distance = 0
        for i in range(len(OTHER)-2):
            p0 = OTHER[i]
            p1 = OTHER[i + 1]
            p2 = OTHER[i + 2]

            vx1 = p1[0] - p0[0]
            vy1 = p1[1] - p0[1]
            mag_v1 = distance_between(p0, p1)

            vx2 = p2[0] - p1[0]
            vy2 = p2[1] - p1[1]
            mag_v2 = distance_between(p1, p2)

            turning += acos((vx1 * vx2 + vy1 * vy2)/(mag_v1 * mag_v2))
        turning = turning/(len(OTHER) - 2)
        print 'turning--->', turning * 34 / 2 / pi

        for i in range(len(OTHER)-1):
            p0 = OTHER[i]
            p1 = OTHER[i + 1]
            distance += distance_between(p0, p1)
        distance = distance/(len(OTHER[0]) - 1)
        print 'distance--->', distance

        p2 = OTHER[-1]
        p1 = OTHER[-2]
        heading = atan2((p2[1] - p1[1]), (p2[0] - p1[0]))

        r = robot(measurement[0], measurement[1], heading, 2*pi / 34.0, 1.5)
        print 'robot--->', r
        r.set_noise(0.01, 0.01, 0)
        r.move_in_circle()
        xy_estimate = r.x, r.y

    return xy_estimate, OTHER



def estimate_next_pos_1(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    xy_estimate = (3.2, 9.1)
    if OTHER is None:
        OTHER = []
        OTHER.append(measurement)
    else:
        OTHER.append(measurement)
        if len(OTHER) >= 3:
            # take the last three stored coordinates in case the first ones are not correct estimates
            point1 = OTHER[len(OTHER) - 3]
            point2 = OTHER[len(OTHER) - 2]
            point3 = OTHER[len(OTHER) - 1]
            y1Delta = point2[1] - point1[1]
            hyp1 = distance_between(point1, point2)
            headingAngle1 = asin(y1Delta / hyp1)

            y2Delta = point3[1] - point2[1]
            hyp2 = distance_between(point2, point3)
            headingAngle2 = asin(y2Delta / hyp2)
            predictedTurnAngle = headingAngle2 - headingAngle1
            newR = robot(point3[0], point3[1], headingAngle2, predictedTurnAngle, hyp1)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y


    return xy_estimate, OTHER


def estimate_next_pos_other(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    if not OTHER:
        opoint = (0.0, 0.0)
        oldHeading = 0.0
        db_a = []
    else:
        opoint = OTHER[0]
        oldHeading = OTHER[1]
        db_a = OTHER[2]
    cpoint = measurement
    delta_y = cpoint[1] - opoint[1]
    delta_x = cpoint[0] - opoint[0]

    newHeading = atan2(delta_y, delta_x)
    db = distance_between(opoint, cpoint)
    db_a.append(db)
    db_a2 = r_[db_a]
    db_m = mean(db_a2)

    turn_angle = newHeading - oldHeading

    heading = newHeading + turn_angle
    x_prime = cpoint[0] + db_m * cos(heading)
    y_prime = cpoint[1] + db_m * sin(heading)

    xy_estimate = (x_prime, y_prime)
    OTHER  = [cpoint, newHeading, db_a]

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
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

        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            return ctr
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
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
    prediction.penup()
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
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized


# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)


angles = []
distances = []
coords = []


def test2():
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while ctr <= 500:
        ctr += 1
        measurement = test_target.sense()
        test_target.move_in_circle()

        if len(coords) == 1:
            x1, y1 = coords[0]
            x2, y2 = measurement
            hypotenuse1 = distance_between(coords[0], measurement)
            y1Delta = y2 - y1
            headingAngleAvg1 = asin(y1Delta / hypotenuse1)
            angles.append(headingAngleAvg1)
            distances.append(hypotenuse1)
        elif len(coords) == 2:
            point1 = coords[0]
            point2 = coords[1]
            point3 = measurement
            y1Delta = point2[1] - point1[1]
            x1Delta = point2[0] - point1[0]
            hypotenuse1 = distance_between(point1, point2)
            headingAngle1 = atan2(y1Delta, x1Delta)
            headingAngleAvg1 = asin(y1Delta / hypotenuse1)
            y2Delta = point3[1] - point2[1]
            x2Delta = point3[0] - point2[0]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = atan2(y2Delta, x2Delta)
            headingAngleAvg2 = asin(y2Delta / hypotenuse2)
            predictedTurnAngle = headingAngle2 - headingAngle1
            predictedTurnAngleAvg = headingAngleAvg2 - headingAngleAvg1
            angles.append(abs(predictedTurnAngleAvg))
            distances.append(hypotenuse2)
        elif len(coords) > 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = measurement
            y1Delta = point2[1] - point1[1]
            x1Delta = point2[0] - point1[0]
            hypotenuse1 = distance_between(point1, point2)
            headingAngle1 = atan2(y1Delta, x1Delta)
            headingAngleAvg1 = asin(y1Delta / hypotenuse1)
            y2Delta = point3[1] - point2[1]
            x2Delta = point3[0] - point2[0]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = atan2(y2Delta, x2Delta)
            headingAngleAvg2 = asin(y2Delta / hypotenuse2)
            predictedTurnAngle = headingAngle2 - headingAngle1
            predictedTurnAngleAvg = headingAngleAvg2 - headingAngleAvg1
            angles.append(abs(predictedTurnAngleAvg))
            distances.append(hypotenuse2)
        coords.append(measurement)

    avgDT = sum(distances)/len(distances)
    avgAngle = sum(angles)/len(angles)
    print angles

    #print "avgDT: ", avgDT
    #print "actual distance: ", 1.5
    print "avgAngle: ", avgAngle
    #print "actual turn angle: ", 2*pi / 34.0



def test():
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while ctr <= 500:
        ctr += 1
        measurement = test_target.sense()
        test_target.move_in_circle()

        if len(coords) == 1:
            x1, y1 = coords[0]
            x2, y2 = measurement
            hypotenuse1 = distance_between(coords[0], measurement)
            y1Delta = y2 - y1
            headingAngle1 = asin(y1Delta / hypotenuse1)
            angles.append(headingAngle1)
            distances.append(hypotenuse1)
        elif len(coords) == 2:
            point1 = coords[0]
            point2 = coords[1]
            point3 = measurement
            y1Delta = point2[1] - point1[1]
            hypotenuse1 = distance_between(point1, point2)
            headingAngle1 = asin(y1Delta / hypotenuse1)
            y2Delta = point3[1] - point2[1]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = asin(y2Delta / hypotenuse2)
            predictedTurnAngle = headingAngle2 - headingAngle1
            angles.append(predictedTurnAngle)
            distances.append(hypotenuse2)
        elif len(coords) > 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = measurement
            y1Delta = point2[1] - point1[1]
            hypotenuse1 = distance_between(point1, point2)
            headingAngle1 = asin(y1Delta / hypotenuse1)
            y2Delta = point3[1] - point2[1]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = asin(y2Delta / hypotenuse2)
            predictedTurnAngle = headingAngle2 - headingAngle1
            angles.append(abs(predictedTurnAngle))
            distances.append(hypotenuse2)

        coords.append(measurement)

    avgDT = sum(distances)/len(distances)
    avgAngle = sum(angles)/len(angles)

    print "avgDT: ", avgDT
    print "actual distance: ", 1.5
    print "avgAngle: ", avgAngle
    print "actual turn angle: ", 2*pi / 34.0

#test2()
#demo_grading_visual(estimate_next_pos, test_target)

scores = []

for i in range(1000):
    scores.append(demo_grading(estimate_next_pos, test_target))

print "average score: ", sum(scores)/len(scores)
print "minimum score: ", min(scores)
print "maximum score: ", max(scores)

#print "actual turn angle: ", 2*pi / 34.0



