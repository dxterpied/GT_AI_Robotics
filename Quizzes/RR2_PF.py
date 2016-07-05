from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random
from numpy import *
import turtle
from collections import Counter

# it appears 4 landmarks is optimal; decreasing landmarks degrades performance; increasing does not seem to have any positive impact
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
size_multiplier= 15.0  #change Size of animation
N = 2000
measurement_noise = 1.0


particles = []
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
test_target.set_noise(0.0, 0.0, 0.05 * test_target.distance)


def createParticles(worldX, worldY, turning, distance):
    # create new particles
    for i in range(N):
        r = robot(random.uniform(worldX - 20, worldX + 20),
                  random.uniform(worldY - 20, worldY + 20),
                  random.random() * 2.0*pi, # noise in orientation
                  turning = turning,
                  distance = distance)
        r.set_noise(new_t_noise = 0.05,
                    new_d_noise = 0.05,
                    new_m_noise = 0.0) # measurement noise is not used in particles

        particles.append(r)



def calculateWeight(particleX, particleY, targetMeasurement):
    # calculates how likely a measurement should be
    prob = 1.0;
    for i in range(len(landmarks)):
        dist = distance_between( (particleX, particleY),  (landmarks[i][0], landmarks[i][1]) )
        prob *= Gaussian(dist, measurement_noise, targetMeasurement[i])
    return prob


# this sense is only used for target bot
def senseToLandmarks(targetX, targetY, measurement_noise):
    Z = []
    import random
    for i in range(len(landmarks)):
        dist = distance_between( (targetX, targetY),  (landmarks[i][0], landmarks[i][1]) )
        Z.append(dist)
    return Z


def Gaussian(mu, sigma, x):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


# returns the arithmetic means of x, y and orientation. It is already weighted.
def get_position(p):
    x = y = 0.0

    # for some reason averages work much better than most common
    # countX = Counter([i.x for i in particles])
    # x = countX.most_common()[0][0]
    #
    # countY = Counter([i.y for i in particles])
    # y = countY.most_common()[0][0]
    #
    # return x, y


    for i in range(len(p)):
        x += p[i].x
        y += p[i].y


    return [ x/len(p), y/len(p) ]


def move(x, y, turning, distance, heading, distance_noise, turning_noise, measurement_noise):
    import random

    newHeading = (heading + turning + random.gauss(0.0, turning_noise)) % (2*pi)
    dist = distance + random.gauss(0.0, distance_noise)
    newX = x + (cos(newHeading) * dist)
    newY = y + (sin(newHeading) * dist)
    # create new particle
    newRobot = robot(newX, newY, newHeading, turning, distance)

    newRobot.set_noise(new_t_noise = turning_noise,
                new_d_noise = distance_noise,
                new_m_noise = measurement_noise) # measurement noise is not used in particles

    return newRobot


"""Computes distance between point1 and point2. Points are (x, y) pairs."""
def distance_between(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def estimate_next_pos(measurement, OTHER = None):

    xy_estimate = measurement

    if OTHER is None:
        distances = []
        angles = []
        coords = []
    else:
        distances, angles, coords = OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], measurement)
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

            # create particles only after approximate turning and distance are known
            if len(particles) == 0:
                createParticles(measurement[0], measurement[1], avgAngle, avgDT)

            Z = senseToLandmarks(measurement[0], measurement[1], 0.05 * test_target.distance)
            xy_estimate = particle_filter(Z, avgAngle, avgDT)

            #print "avgAngle:", avgAngle
            newR = robot(xy_estimate[0], xy_estimate[1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y

        #print "headingAngle1", headingAngle1

    coords.append(measurement)
    OTHER = (distances, angles, coords)

    return xy_estimate, OTHER


def particle_filter(targetMeasurementToLandmarks, averageTurning, averageDistance):
    global particles

    # PREDICT by moving
    p2 = []
    for i in range(N):
        newParticle = move(particles[i].x, particles[i].y, averageTurning, averageDistance, particles[i].heading, particles[i].distance_noise, particles[i].turning_noise, particles[i].measurement_noise)
        p2.append(newParticle)
    particles = p2

    # UPDATE by creating weights
    w = []
    for i in range(N):
        particle = particles[i]
        mp = calculateWeight( particle.x, particle.y, targetMeasurementToLandmarks)
        w.append(  mp )

    # RESAMPLING
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append( particles[index] )
    particles = p3
    # end resampling

    return get_position(particles)



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

            # countX = Counter([i.x for i in particles])
            # x = countX.most_common()[0][0]
            # countY = Counter([i.y for i in particles])
            # y = countY.most_common()[0][0]
            # print "x, y", x, y
            # print "true_position", true_position
            # print "position_guess", position_guess

            print "You got it right! It took you ", ctr, " steps to localize."

            # prediction.color('red')
            # for i in range(N):
            #     p = particles[i]
            #     prediction.goto(p.x * size_multiplier, p.y  *size_multiplier-200)
            #     prediction.stamp()


            return ctr
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


# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.


demo_grading_visual(estimate_next_pos, test_target)
#demo_grading(estimate_next_pos, test_target)

# scores = []
# fails = 0
# for i in range(1000):
#
#     particles = []
#     test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
#     test_target.set_noise(0.0, 0.0, 0.05 * test_target.distance)
#     particles = []
#     score = demo_grading(estimate_next_pos, test_target)
#
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

# 100 runs:
# average score:  103.04
# minimum score:  7
# maximum score:  291
# fails:  0


#turtle.getscreen()._root.mainloop()





