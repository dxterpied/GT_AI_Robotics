from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random
from numpy import *


landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
particles = []
world_size = 100.0
bearing_noise = 0.05

#create particles for the state space
for x in range(-10, 10):
    for y in range(0, 30):

        r = robot(x, y, 0.0, 2*pi / 30, 1.5)
        r.set_noise(0.05, 0.05, 5.0)
        particles.append(r)


def measurement_prob(x, y, orientation, bearing_noise, measurement):
    # calculate the correct measurement
    predicted_measurements = sense(x, y, orientation, 0) # Our sense function took 0 as an argument to switch off noise.
    # compute errors
    error = 1.0
    error_bearing = abs(measurement - predicted_measurements)
    error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
    # update Gaussian
    error *= (exp(- (error_bearing ** 2) / (bearing_noise ** 2) / 2.0) /
              sqrt(2.0 * pi * (bearing_noise ** 2)))
    return error


def sense(x, y, orientation, bearing_noise, noise = 1): #do not change the name of this function
    Z = []
    import random

    for landmark in landmarks:
        ly, lx = landmark
        headingToLandmark = atan2( ly - y , lx - x )
        bearing = headingToLandmark - orientation
        if noise == 1:
            bearing += random.gauss(0, bearing_noise)

        Z.append( bearing % (2*pi)  )
    return Z #Leave this line here. Return vector Z of 4 bearings.


def particle_filter(measurement, p):
    # Update particles
    p2 = []
    N = len(p)
    for i in range(N):
        p2.append(p[i].move_in_circle())
    p = p2

    # measurement update
    w = []
    for i in range(N):
        w.append(measurement_prob(p[i].x, p[i].y, p[i].heading, bearing_noise, measurement))

    # resampling
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3

    return get_position(p)

# returns the arithmetic means of x, y and orientation. It is already weighted.
def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi))
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]


def estimate_next_pos(measurement, OTHER = None):
    global particles
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

        elif len(coords) >= 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = measurement

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

            z = sense(x, y, headingAngle2, bearing_noise, noise = 1)
            particle_filter(z, particles)

            #print "avgDT:", avgDT, "avgAngle:", avgAngle, "headingAngle2:", headingAngle2

            #print "avgAngle:", avgAngle
            newR = robot(point3[0], point3[1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y
        #print "headingAngle1", headingAngle1

    coords.append(measurement)
    OTHER = (distances, angles, coords)

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




def test2():
    angles = []
    distances = []
    coords = []

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




demo_grading_visual(estimate_next_pos, test_target)
#demo_grading(estimate_next_pos, test_target)
# scores = []
# for i in range(10000):
#     scores.append(demo_grading(estimate_next_pos, test_target))
#
# print "average score: ", sum(scores)/len(scores)
# print "minimum score: ", min(scores)
# print "maximum score: ", max(scores)

# stats:
# Sorry, it took you too many steps to localize the target.
# Sorry, it took you too many steps to localize the target.
# Sorry, it took you too many steps to localize the target.
# average score:  123
# minimum score:  False
# maximum score:  947


#print "actual turn angle: ", 2*pi / 34.0



