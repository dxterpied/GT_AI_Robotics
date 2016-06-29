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
world_size = 10.0
measurement_noise = 0.05 * 1.5
bearing_noise = 0.1


#create randomly distributed particles
# random distribution close to the original location seems to give better results than even particle distribution
# where there is turning and distance noise, the results are worse than without noise
# using more than 1000 particles did not make any difference
# applying heading to weight calculation made it much worse
for i in range(1000):
    # r = robot(0.0, 0.0, 0.5, 2*pi / 34.0, 1.5)

    # r = robot(random.uniform(-1, 1) * world_size,
    #           random.uniform(-1, 1) * world_size,
    #           0.5 + random.random(),
    #           2*pi/34.0 + random.random(),
    #           1.5 + random.random())
    # r.set_noise(0.01, 0.01, measurement_noise)

    r = robot(random.uniform(-1, 1) * world_size,
              random.uniform(-1, 1) * world_size,
              0.5,
              2*pi/34.0,
              1.5)
    # r.set_noise(0.0, 0.0, measurement_noise)
    r.set_noise(0.0, 0.0, 0.0)

    particles.append(r)


#create evenly distributed particles for the state space
# for x in range(-10, 10):
#     for y in range(-10, 10):
#         # add two particles per x, y
#         r = robot(x, y, random.random() * 2.0 * pi, 2*pi / 34.0, 1.5)
#         r.set_noise(0.0, 0.0, measurement_noise)
#         particles.append(r)
#
#         r = robot(x, y, random.random() * 2.0 * pi, 2*pi / 34.0, 1.5)
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


def measurement_probHeading(particleX, particleY, particleOrientation, targetMeasurementHeading):
    predicted_measurements = senseHeading(particleX, particleY, particleOrientation)

    # compute errors
    error = 1.0
    for i in range(len(targetMeasurementHeading)):
        error_bearing = abs(targetMeasurementHeading[i] - predicted_measurements[i])
        #print error_bearing 2.30763730938

        error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
        # update Gaussian
        error *= (exp(- (error_bearing ** 2) / (bearing_noise ** 2) / 2.0) /
                  sqrt(2.0 * pi * (bearing_noise ** 2)))
    return error

def sense(targetX, targetY):
    Z = []
    import random

    for i in range(len(landmarks)):
        dist = distance_between( (targetX, targetY),  (landmarks[i][0], landmarks[i][1]) )
        dist += random.gauss(0.0, measurement_noise)
        Z.append(dist)
    return Z


def senseHeading(x, y, orientation, noise = 1): #do not change the name of this function
    Z = []

    for landmark in landmarks:
        ly, lx = landmark
        headingToLandmark = atan2( ly - y , lx - x )
        bearing = headingToLandmark - orientation
        Z.append( bearing % (2*pi)  )
    return Z #Leave this line here. Return vector Z of 4 bearings. For example: [1.9267312016649392, 0.4641086737309461, 5.618444871041761, 4.514357510936977]



def Gaussian(mu, sigma, x):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


def particle_filter(targetMeasurementToLandmarks, targetMeasurementHeading, p):

    # PREDICTION
    N = len(p)
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

    # dataX = Counter([item.x for item in p])
    # dataY = Counter([item.y for item in p])
    # dataOrientation = Counter([item.heading for item in p])
    #
    # dataX = dataX.most_common(1)
    # dataY = dataY.most_common(1)
    # dataOrientation = dataOrientation.most_common(1)

    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        orientation += (((p[i].heading - p[0].heading + pi) % (2.0 * pi)) + p[0].heading - pi)
    return [x / len(p), y / len(p), orientation / len(p)]
    #return [dataX[0][0], dataY[0][0], dataOrientation[0][0]]




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
            distances.append(hypotenuse2)
            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)
            # sense bearings to landmarks by target robot

            z = sense(measurement[0], measurement[1])
            headingToLandmarks = senseHeading(measurement[0], measurement[1], headingAngle2)
            predictedX, predictedY, predictedHeading = particle_filter(z, headingToLandmarks, particles)

            # predictedTurnAngleAvg = headingAngle2 - headingAngle1
            # angles.append(abs(predictedTurnAngleAvg))

            newR = robot(predictedX, predictedY, headingAngle2, 2*pi/34.0, 1.5)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y

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

        #print "true_position", true_position
        #print "position_guess", position_guess

        error = distance_between(position_guess, true_position)

        print "error", error
        #print "distance_tolerance", distance_tolerance


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


test_target = robot(0.0, 0.0, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

#demo_grading_visual(estimate_next_pos, test_target)
#demo_grading(estimate_next_pos, test_target)

scores = []
fails = 0
for i in range(100):
    test_target = robot(0.0, 0.0, 0.5, 2*pi / 34.0, 1.5)
    test_target.set_noise(0.0, 0.0, measurement_noise)
    score = demo_grading(estimate_next_pos, test_target)

    for i in range(1000):
        r = robot(random.uniform(-1, 1) * world_size,
                  random.uniform(-1, 1) * world_size,
                  0.5,
                  2*pi/34.0,
                  1.5)
        r.set_noise(0.0, 0.0, 0.0)
        particles.append(r)
    if score == 1000:
        fails += 1
    else:
        scores.append(score)

print "average score: ", sum(scores)/ float(len(scores))
print "minimum score: ", min(scores)
print "maximum score: ", max(scores)
print "fails: ", fails


#turtle.getscreen()._root.mainloop()




