from robot import *
from math import *
from matrix import *
import random
from numpy import *


landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
particles = []
world_size = 100.0
bearing_noise = 0.05
distance_noise = 0.05
measurement_noise = 5.0

#create evenly distributed particles for the state space
for x in range(-10, 10):
    for y in range(0, 30):
        # add two particles per x, y
        r = robot(x, y, 0.0, 2*pi / 30, 1.5)
        r.set_noise(bearing_noise, distance_noise, measurement_noise)
        particles.append(r)
        r = robot(x, y, 0.0, 2*pi / 30, 1.5)
        r.set_noise(bearing_noise, distance_noise, measurement_noise)
        particles.append(r)
        r = robot(x, y, 0.0, 2*pi / 30, 1.5)
        r.set_noise(bearing_noise, distance_noise, measurement_noise)
        particles.append(r)


def measurement_prob(particleX, particleY, particleOrientation, bearing_noise, targetMeasurementsToLandmarks):
    # print targetMeasurementsToLandmarks [3.3248981415499257, 2.3415178749937997, 0.7312337564877514, 4.864542506564464]
    # calculate the correct measurement
    predicted_measurements = sense(particleX, particleY, particleOrientation)

    error = 1.0
    # compute errors
    # go through each measurement to a landmark
    for i in range(len(targetMeasurementsToLandmarks)):
        error_bearing = abs(targetMeasurementsToLandmarks[i] - predicted_measurements[i])
        error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
        # update Gaussian
        error *= (exp(- (error_bearing ** 2) / (bearing_noise ** 2) / 2.0) /
                  sqrt(2.0 * pi * (bearing_noise ** 2)))

    return error

def measurement_prob2(particleX, particleY, targetMeasurement):
    # calculates how likely a measurement should be
    prob = 1.0;
    for i in range(len(landmarks)):
        dist = sqrt((particleX - landmarks[i][0]) ** 2 + (particleY - landmarks[i][1]) ** 2)
        prob *= Gaussian(dist, measurement_noise, targetMeasurement[i])
    return prob




def Gaussian(mu, sigma, x):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


# senses bearings (turning angle) to landmarks (with or without noise). Is used by both target robot and particles
# particles sense without noise, target senses with noise?
def sense(x, y, orientation):
    Z = []
    import random

    for landmark in landmarks:
        ly, lx = landmark
        headingToLandmark = atan2( ly - y , lx - x )
        bearing = headingToLandmark - orientation
        bearing += random.gauss(0, 0.05)
        Z.append( bearing % (2*pi)  )

    return Z #Return vector of 4 bearings. For example: [1.9267312016649392, 0.4641086737309461, 5.618444871041761, 4.514357510936977]


def sense2(targetX, targetY):
    Z = []
    import random

    for i in range(len(landmarks)):
        dist = sqrt((targetX - landmarks[i][0]) ** 2 + (targetY - landmarks[i][1]) ** 2)
        dist += random.gauss(0.0, measurement_noise)
        Z.append(dist)
    return Z



def particle_filter2(targetMeasurementToLandmarks, p):

    # Update particles
    p2 = []
    N = len(p)
    for i in range(N):
        # move the particle the same way the target would move
        p[i].move_in_circle()

    # measurement update
    w = []
    for i in range(N):
        particle = p[i]
        w.append(  measurement_prob2( particle.x, particle.y, targetMeasurementToLandmarks) )

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
        p3.append( p[index] )
    p = p3


    return get_position(p)


# returns x, y, heading (orientation)  [108.67750354104683, 125.76791503810604, 2.019868233686369]
def particle_filter(targetMeasurementToLandmarks, p):

    # Update particles
    p2 = []
    N = len(p)
    for i in range(N):
        # move the particle the same way the target would move
        p[i].move_in_circle()

    # measurement update
    w = []
    for i in range(N):
        particle = p[i]
        w.append(  measurement_prob( particle.x, particle.y, particle.heading, bearing_noise, targetMeasurementToLandmarks) )

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
        p3.append( p[index] )
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
        orientation += (((p[i].heading - p[0].heading + pi) % (2.0 * pi))
                        + p[0].heading - pi)
    return [x / len(p), y / len(p), orientation / len(p)]

def estimate_next_pos2(measurement, OTHER = None):
    global particles

    xy_estimate = (0., 0.)

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
            x1Delta = point2[0] - point1[0]
            headingAngle1 = atan2(y1Delta, x1Delta)

            y2Delta = point3[1] - point2[1]
            x2Delta = point3[0] - point2[0]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = atan2(y2Delta, x2Delta)

            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            # sense bearings to landmarks by target robot
            z = sense2(measurement[0], measurement[1])
            predictedX, predictedY, predictedHeading = particle_filter2(z, particles)

            predictedTurnAngleAvg = headingAngle2 - headingAngle1
            angles.append(abs(predictedTurnAngleAvg))


            #print predictedX, predictedY, predictedHeading
            #print measurement[0], measurement[1], headingAngle2

            #print "avgDT:", avgDT, "avgAngle:", avgAngle, "headingAngle2:", headingAngle2
            #print "avgAngle:", avgAngle

            #newR = robot(measurement[0], measurement[1], headingAngle2, avgAngle, avgDT)
            newR = robot(predictedX, predictedY, predictedHeading, 2*pi / 30, 1.5)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y


        #print "headingAngle1", headingAngle1

    coords.append(measurement)
    OTHER = (distances, angles, coords)

    return xy_estimate, OTHER


def estimate_next_pos(measurement, OTHER = None):
    global particles
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    xy_estimate = (0., 0.)

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
            x1Delta = point2[0] - point1[0]
            headingAngle1 = atan2(y1Delta, x1Delta)

            y2Delta = point3[1] - point2[1]
            x2Delta = point3[0] - point2[0]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = atan2(y2Delta, x2Delta)

            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            # sense bearings to landmarks by target robot
            z = sense(measurement[0], measurement[1], headingAngle2)
            predictedX, predictedY, predictedHeading = particle_filter(z, particles)

            predictedTurnAngleAvg = headingAngle2 - headingAngle1
            angles.append(abs(predictedTurnAngleAvg))


            #print predictedX, predictedY, predictedHeading
            #print measurement[0], measurement[1], headingAngle2

            #print "avgDT:", avgDT, "avgAngle:", avgAngle, "headingAngle2:", headingAngle2
            #print "avgAngle:", avgAngle

            #newR = robot(measurement[0], measurement[1], headingAngle2, avgAngle, avgDT)
            newR = robot(predictedX, predictedY, predictedHeading, 2*pi / 30, 1.5)
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
            print "True position: ", true_position
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


#demo_grading_visual(estimate_next_pos, test_target)
demo_grading_visual(estimate_next_pos2, test_target)

#demo_grading(estimate_next_pos, test_target)
# scores = []
# for i in range(10000):
#     scores.append(demo_grading(estimate_next_pos, test_target))
#
# print "average score: ", sum(scores)/len(scores)
# print "minimum score: ", min(scores)
# print "maximum score: ", max(scores)






