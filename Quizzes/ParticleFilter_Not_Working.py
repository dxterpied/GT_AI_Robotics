from robot import *
from math import *
from matrix import *
import random
from numpy import *
from collections import Counter
import time
import turtle

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
particles = []
world_size = 3.0
measurement_noise = 0.05 * 1.5
bearing_noise = 0.1


# window = turtle.Screen()
# window.screensize(800, 800)
# window.bgcolor('white')
# size_multiplier= 25.0  #change Size of animation
# particle = turtle.Turtle()
# particle.shape('arrow')
# particle.color('red')
# particle.resizemode('user')
# particle.shapesize(0.4, 0.4, 0.4)
# particle.penup()


#create randomly distributed particles
# random distribution close to the original location seems to give better results than even particle distribution
# where there is turning and distance noise, the results are worse than without noise
# using more than 1000 particles did not make any difference
# applying heading to weight calculation made it much worse
#for i in range(1000):
    # r = robot(random.uniform(-1, 1) * world_size,
    #           random.uniform(-1, 1) * world_size,
    #           0.5 + random.random(),
    #           2*pi/34.0 + random.random(),
    #           1.5 + random.random())
    # r.set_noise(0.01, 0.01, measurement_noise)

    # this  works better than the other two methods
    # but it is not right because it only works if heading, turning and distance match those of target
    # r = robot(random.uniform(-1, 1) * world_size,
    #           random.uniform(-1, 1) * world_size,
    #           0.5,
    #           2*pi/34.0,
    #           1.5)
    # r.set_noise(0.0, 0.0, 0.0)


    # random x, y, orientation
    # r = robot(random.uniform(-1, 1) * world_size,
    #           random.uniform(-1, 1) * world_size,
    #           heading = random.random() * 2.0*pi, # noise in orientation
    #           turning = 2*pi/34.0,
    #           distance = 1.5)
    # r.set_noise(new_t_noise = 0.05,
    #             new_d_noise = 0.05,
    #             new_m_noise = measurement_noise) # measurement noise is not used in particles

    # particle.setheading(r.heading * 180/pi)
    # particle.goto(r.x*size_multiplier, r.y*size_multiplier-200)
    # particle.stamp()

    #particles.append(r)


#create evenly distributed particles for the state space
# for x in range(-10, 20):
#     for y in range(-10, 20):
#         r = robot(x, y, 0.5, 2*pi/34.0, 1.5)
#         r.set_noise(0.0, 0.0, measurement_noise)
#         particles.append(r)


#print len(particles)

def measurement_prob(particleX, particleY, targetMeasurement, targetMeasurementHeading = 0):
    # calculates how likely a measurement should be
    prob = 1.0;
    for i in range(len(landmarks)):
        dist = distance_between( (particleX, particleY),  (landmarks[i][0], landmarks[i][1]) )
        prob *= Gaussian(dist, measurement_noise, targetMeasurement[i])
    return prob


# this sense is only used for target bot, hence no randomness
def sense(targetX, targetY):
    Z = []
    import random

    for i in range(len(landmarks)):
        dist = distance_between( (targetX, targetY),  (landmarks[i][0], landmarks[i][1]) )
        # dist += random.gauss(0.0, measurement_noise) no noise is needed because particles should have no measurement noise and
        # target already has noise when giving its x and y coordinates
        Z.append(dist)
    return Z


def Gaussian(mu, sigma, x):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


def particle_filter(targetMeasurementToLandmarks, targetMeasurementHeading, p):

    # PREDICTION
    N = len(p)

    # this makes it dance all over the place
    # p2 = []
    # for i in range(N):
    #     newR = robot(p[i].x, p[i].y, p[i].heading, p[i].turning, p[i].distance)
    #     newR.set_noise(p[i].turning_noise, p[i].distance_noise, p[i].measurement_noise)
    #     newR.move_in_circle()
    #     p2.append(newR)
    # p = p2


    for i in range(N):
        # move the particle the same way the target would move
        p[i].move_in_circle()

    # UPDATE
    w = []
    for i in range(N):
        particle = p[i]
        w.append(  measurement_prob( particle.x, particle.y, targetMeasurementToLandmarks) )

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
        orientation += (((p[i].heading - p[0].heading + pi) % (2.0 * pi)) + p[0].heading - pi)
    return [x / len(p), y / len(p), orientation / len(p)]


def estimate_next_pos(measurement, OTHER = None):
    global particles

    xy_estimate = measurement

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

            #print "headingAngle2", headingAngle2 this angle is pretty close to the actual angle

            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)

            #avgAngle = sum(angles)/len(angles)
            # sense bearings to landmarks by target robot

            z = sense(measurement[0], measurement[1])
            # #headingToLandmarks = senseHeading(measurement[0], measurement[1], headingAngle2)
            predictedX, predictedY, predictedHeading = particle_filter(z, None, particles)

            # predictedTurnAngleAvg = headingAngle2 - headingAngle1
            # angles.append(abs(predictedTurnAngleAvg))

            newR = robot(predictedX, predictedY, headingAngle2, 2*pi/34.0, 1.5) # create a bot with guessed coordinates x, y, heading
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y

    # z = sense(measurement[0], measurement[1])
    # predictedX, predictedY, predictedHeading = particle_filter(z, None, particles)
    # xy_estimate = predictedX, predictedY

    coords.append(measurement)
    OTHER = (distances, angles, coords)

    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


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

        #print "error", error

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
    print "distance_tolerance", distance_tolerance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.screensize(800, 800)
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
    prediction.shape('circle')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        #time.sleep(1)
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
        # measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        # measured_broken_robot.stamp()
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()



        #End of Visualization
    return localized


target_bot = robot(0.0, 0.0, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * target_bot.distance
target_bot.set_noise(0.0, 0.0, measurement_noise)

demo_grading_visual(estimate_next_pos, target_bot)
#demo_grading(estimate_next_pos, target_bot)

# scores = []
# fails = 0
# for i in range(100):
#     target_bot = robot(0.0, 0.0, 0.5, 2*pi / 34.0, 1.5)
#     target_bot.set_noise(0.0, 0.0, measurement_noise)
#
#     particles = []
#     for i in range(1000):
#
#         r = robot(random.uniform(-1, 1) * world_size,
#               random.uniform(-1, 1) * world_size,
#               heading = random.random() * 2.0*pi, # noise in orientation
#               turning = 2*pi/34.0,
#               distance = 1.5)
#
#         # r = robot(random.uniform(-1, 1) * world_size,
#         #           random.uniform(-1, 1) * world_size,
#         #           0.5,
#         #           2*pi/34.0,
#         #           1.5)
#         # r.set_noise(0.0, 0.0, 0.0)
#         particles.append(r)
#
#     score = demo_grading(estimate_next_pos, target_bot)
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


turtle.getscreen()._root.mainloop()




