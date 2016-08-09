from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random
from numpy import *
import turtle
from collections import Counter

# Ilya: this is based on RR5_PF  but attempts to improve by adding fresh random particles
# unsuccessful as well.........

# it appears 4 landmarks is optimal; decreasing landmarks degrades performance; increasing does not seem to have any positive impact
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
size_multiplier= 20.0  #change Size of animation
N = 100
measurement_noise = 1.0
particles = []

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
target.set_noise(0.0, 0.0, 2. * target.distance)
hunter = robot(-10.0, -10.0, 0.0)

# bumblebee = turtle.Turtle()
# bumblebee.shape('square')
# bumblebee.color('yellow')
# bumblebee.shapesize(0.2, 0.2, 0.2)



def least_squares(x, y, x_actual = None, y_actual = None, show_plot = False):

    from matplotlib import pyplot as p
    x = r_[x]
    y = r_[y]
    if x_actual is not None:
        x_actual = r_[x_actual]
    if y_actual is not None:
        y_actual = r_[y_actual]
    # coordinates of the barycenter
    x_m = mean(x)
    y_m = mean(y)
    # calculation of the reduced coordinates
    u = x - x_m
    v = y - y_m

    # linear system defining the center in reduced coordinates (uc, vc):
    #    Suu * uc +  Suv * vc = (Suuu + Suvv)/2
    #    Suv * uc +  Svv * vc = (Suuv + Svvv)/2
    Suv  = sum(u*v)
    Suu  = sum(u**2)
    Svv  = sum(v**2)
    Suuv = sum(u**2 * v)
    Suvv = sum(u * v**2)
    Suuu = sum(u**3)
    Svvv = sum(v**3)

    # Solving the linear system
    A = array([ [ Suu, Suv ], [Suv, Svv]])
    B = array([ Suuu + Suvv, Svvv + Suuv ])/2.0
    uc, vc = linalg.solve(A, B)
    # center coordinates
    xc = x_m + uc
    yc = y_m + vc
    # Calculation of all distances from the center (xc_1, yc_1)
    Ri_1      = sqrt((x - xc)**2 + (y - yc)**2) # distance of given points from center
    radius    = mean(Ri_1)

    if show_plot:
        theta_fit = linspace(-pi, pi, 180)
        x_fit = xc + radius * cos(theta_fit)
        y_fit = yc + radius * sin(theta_fit)
        # center
        p.plot([xc], [yc], 'bD', mec='y', mew=1)
        # calculated circle
        p.plot(x_fit, y_fit, label="calculated", lw=2)
        if x_actual is not None:
            # actual circle points
            p.plot(x_actual, y_actual, color='black', label='actual', ms=8, mec='b', mew=1)
        # data points given
        p.plot(x, y, 'ro', label='data', ms=8, mec='b', mew=1)
        p.legend(loc='best',labelspacing=0.1 )
        p.grid()
        p.xlabel('x')
        p.ylabel('y')
        p.title('Least Squares Circle')
        p.savefig("circle_png")
        p.show()


    return radius, xc, yc


# calculate the average turn angle
def getTurnAngle(measurements, rotationSign, xc, yc):
    angle = 0.

    # get the very first heading angle (measured). It's a ball park to get started
    xDelta = measurements[0][0] - xc
    yDelta = measurements[0][1] - yc
    firstHeading = atan2(yDelta, xDelta)

    #print "firstHeading", firstHeading

    prevHeading = firstHeading
    totalAngle = 0.

    # if len(measurements) > 25:
    #     print "currentHeading - prevHeading = turningAngle", "rotationSign", rotationSign
    #print "len(measurements[1:])", len(measurements[1:])
    for coords in measurements[1:]:
        x, y = coords
        # get heading to measurement
        xDelta = x - xc
        yDelta = y - yc
        currentHeading = atan2(yDelta, xDelta)

        # difference between current and previous
        if currentHeading < 0. and abs(currentHeading) > pi/2 and prevHeading > 0. and prevHeading > pi/2:
            turningAngle = 2 * pi + currentHeading - prevHeading
        elif currentHeading > 0. and currentHeading > pi/2 and prevHeading < 0. and abs(prevHeading) > pi/2:
            turningAngle = -(2 * pi + currentHeading - prevHeading)
        else:
            turningAngle = currentHeading - prevHeading

        # if len(measurements) > 25:
        #     print currentHeading, "\t-", "(", prevHeading, ")", "\t=", turningAngle

        # if len(measurements[1:]) > 40:
        #     print "turningAngle", turningAngle

        if (turningAngle * rotationSign) > 0: # if signs match, it means rotation in the same direction
            # if len(measurements[1:]) > 40:
            #     print "\ttotalAngle", totalAngle
            totalAngle += abs(turningAngle)
            # if len(measurements) > 25:
            #     print "\t\ttotalAngle", totalAngle

            # previous can only become current if the right angle is added
            prevHeading = currentHeading


    # if (totalAngle / pi) >= 2:
    #     print "\t", totalAngle,  totalAngle / steps

    #print "totalAngle", totalAngle
    angle = abs(totalAngle / len(measurements))
    #print "angle", angle

    # if angle > .27:
    #     exit()
    return angle_trunc(angle), angle_trunc(totalAngle)


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


def next_move_straight_line(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    numberOfSkippedSteps = 20
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        xy_pf = (0, 0)
        turnAngle = []
        x = []
        x.append(target_measurement[0])
        y = []
        y.append(target_measurement[1])

    else:
        distances, angles, coords, xy_estimate, steps, xy_pf, turnAngle, x, y = OTHER
        # collect measurements
        x.append(target_measurement[0])
        y.append(target_measurement[1])

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
            distances.append(hypotenuse1)
            xy_estimate = target_measurement
        elif len(coords) >= 2 and len(coords) <= numberOfSkippedSteps:
            radius, xc, yc = least_squares(x, y) # actual radius is 7.175; this estimate is about 7.62; that's bad but we don't have anything better...
            xcDelta = target_measurement[0] - xc
            ycDelta = target_measurement[1] - yc
            angle = angle_trunc(atan2(ycDelta, xcDelta))
            estimated_x = xc + radius * cos(angle)
            estimated_y = yc + radius * sin(angle)
            xy_estimate = estimated_x, estimated_y

        elif len(coords) > numberOfSkippedSteps:
            point1 = coords[len(coords) - 16]
            point2 = coords[len(coords) - 8]
            point3 = target_measurement

            rotationDirection = calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1])
            turnAngle.append(rotationDirection)
            rotationSign = getRotationSign(turnAngle)

            # estimate radius and center using least squares
            radius, xc, yc = least_squares(x, y) # actual radius is 7.175; this estimate is about 7.62; that's bad but we don't have anything better...
            #print "radius", radius # prints about 7.62; actual is 7.175
            # get estimated turning and total angle traveled from measured start
            turning, totalAngle = getTurnAngle(coords, rotationSign, xc, yc)
            #print "turning", turning # prints about 0.21; actual is 0.2
            distance = 2 * radius * sin(turning/2) # chord distance calculation
            #print "distance", distance # gives about 1.65; correct one is 1.5

            # create particles only after approximate turning and distance are known
            if len(particles) == 0:
                # create particles based on the first predicted location
                x0Delta = x[0] - xc # using first measured x
                y0Delta = y[1] - yc # using first measured y
                angle = angle_trunc(atan2(y0Delta, x0Delta)) # first heading from the predicted center based on the first measurement
                # put the first measured point on the estimated circumference
                estimated_x = xc + radius * cos(angle)
                estimated_y = yc + radius * sin(angle)
                # create particles with the starting location of the first predicted point
                createParticles(estimated_x, estimated_y, rotationSign * turning, distance, N)

                # now, advance this for the number of skipped steps to catch up the estimations
                for i in range(numberOfSkippedSteps):
                    Z = senseToLandmarks(estimated_x, estimated_y)
                    xy_pf = particle_filter(Z, rotationSign * turning, distance, None, None)
                    # get new estimated measurements based on the predicted turn angle and distance (not actual measurements)
                    angle = angle_trunc(angle + (rotationSign * turning))
                    estimated_x = xc + radius * cos(angle)
                    estimated_y = yc + radius * sin(angle)
            else:
                xcDelta = target_measurement[0] - xc
                ycDelta = target_measurement[1] - yc
                angle = angle_trunc(atan2(ycDelta, xcDelta))
                estimated_x = xc + radius * cos(angle)
                estimated_y = yc + radius * sin(angle)

                Z = senseToLandmarks(estimated_x, estimated_y)
                xy_pf = particle_filter(Z, rotationSign * turning, distance, estimated_x, estimated_y)

                xcDelta = xy_pf[0] - xc
                ycDelta = xy_pf[1] - yc
                totalAngle = angle_trunc(atan2(ycDelta, xcDelta))

            # bumblebee.goto(xy_pf[0] * size_multiplier, xy_pf[1] * size_multiplier - 200)
            # bumblebee.stamp()

            newR = robot(xy_pf[0], xy_pf[1], totalAngle, rotationSign * turning, distance)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y

            steps = 1
            while True:
                # check how many steps it will take to get there for Hunter
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1
                newR.move_in_circle()
                xy_estimate = newR.x, newR.y

            # steps = 1
            # while True:
            #     # check how many steps it will take to get there for Hunter
            #     if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
            #         break
            #     steps += 1
            #
            #     totalAngle += rotationSign * turning
            #     estimated_x = xc + radius * cos(totalAngle)
            #     estimated_y = yc + radius * sin(totalAngle)
            #     xy_estimate = estimated_x, estimated_y

            # put this estimate on the predicted circumference
            xDelta = newR.x - xc
            yDelta = newR.y - yc
            angle = angle_trunc(atan2(yDelta, xDelta)) # first heading from the predicted center based on the first measurement
            # put the first measured point on the estimated circumference
            estimated_x = xc + radius * cos(angle)
            estimated_y = yc + radius * sin(angle)
            xy_estimate = estimated_x, estimated_y



    coords.append(target_measurement)

    OTHER = (distances, angles, coords, xy_estimate, steps, xy_pf, turnAngle, x, y)
    if xy_estimate is None:
        xy_estimate = target_measurement
    turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading) # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)

    return turning, distance, OTHER


def next_move_straight_line_1(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    numberOfSkippedSteps = 30

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        xy_pf = (0, 0)
        turnAngle = []
        x = []
        x.append(target_measurement[0])
        y = []
        y.append(target_measurement[1])

    else:
        distances, angles, coords, xy_estimate, steps, xy_pf, turnAngle, x, y = OTHER

        # collect measurements
        x.append(target_measurement[0])
        y.append(target_measurement[1])

        xy_estimate = target_measurement

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
            distances.append(hypotenuse1)
            xy_estimate = target_measurement

        elif len(coords) > numberOfSkippedSteps:
            point1 = coords[len(coords) - 16]
            point2 = coords[len(coords) - 8]
            point3 = target_measurement

            rotationDirection = calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1])
            turnAngle.append(rotationDirection)
            rotationSign = getRotationSign(turnAngle)

            # estimate radius and center using least squares
            radius, xc, yc = least_squares(x, y) # actual radius is 7.175; this estimate is about 7.62; that's bad but we don't have anything better...
            #print "radius", radius # prints about 7.62; actual is 7.175
            # get estimated turning and total angle traveled from measured start
            turning, totalAngle = getTurnAngle(coords, rotationSign, xc, yc)
            #print "turning", turning # prints about 0.21; actual is 0.2
            distance = 2 * radius * sin(turning/2) # chord distance calculation
            #print "distance", distance # gives about 1.65; correct one is 1.5



            # create particles only after approximate turning and distance are known
            if len(particles) == 0:

                # create particles based on the first predicted location
                x0Delta = x[0] - xc # using first measured x
                y0Delta = y[1] - yc # using first measured y
                angle = angle_trunc(atan2(y0Delta, x0Delta)) # first heading from the predicted center based on the first measurement
                # put the first measured point on the estimated circumference
                estimated_x = xc + radius * cos(angle)
                estimated_y = yc + radius * sin(angle)
                # create particles with the starting location of the first predicted point
                createParticles(estimated_x, estimated_y, rotationSign * turning, distance)

                # now, advance this for the number of skipped steps to catch up the estimations
                for i in range(numberOfSkippedSteps):
                    Z = senseToLandmarks(estimated_x, estimated_y)
                    xy_pf = particle_filter(Z, rotationSign * turning, distance)
                    # get new estimated measurements based on the predicted turn angle and distance (not actual measurements)
                    angle = angle_trunc(angle + (rotationSign * turning))
                    estimated_x = xc + radius * cos(angle)
                    estimated_y = yc + radius * sin(angle)


            else:
                xcDelta = target_measurement[0] - xc
                ycDelta = target_measurement[1] - yc
                angle = angle_trunc(atan2(ycDelta, xcDelta))
                estimated_x = xc + radius * cos(angle)
                estimated_y = yc + radius * sin(angle)

                Z = senseToLandmarks(estimated_x, estimated_y)
                xy_pf = particle_filter(Z, rotationSign * turning, distance)

                xcDelta = xy_pf[0] - xc
                ycDelta = xy_pf[1] - yc
                totalAngle = angle_trunc(atan2(ycDelta, xcDelta))

            # bumblebee.goto(xy_pf[0] * size_multiplier, xy_pf[1] * size_multiplier - 200)
            # bumblebee.stamp()

            newR = robot(xy_pf[0], xy_pf[1], totalAngle, rotationSign * turning, distance)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y

            # try to find the shortest straight path from hunter position to predicted target position
            steps = 1
            while True:
                # check how many steps it will take to get there for Hunter
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1
                newR.move_in_circle()

                # put this estimate on the predicted circumference
                xDelta = newR.x - xc
                yDelta = newR.y - yc
                angle = angle_trunc(atan2(yDelta, xDelta)) # first heading from the predicted center based on the first measurement
                # put the first measured point on the estimated circumference
                estimated_x = xc + radius * cos(angle)
                estimated_y = yc + radius * sin(angle)
                xy_estimate = estimated_x, estimated_y


            # put this estimate on the predicted circumference
            xDelta = newR.x - xc
            yDelta = newR.y - yc
            angle = angle_trunc(atan2(yDelta, xDelta)) # first heading from the predicted center based on the first measurement
            # put the first measured point on the estimated circumference
            estimated_x = xc + radius * cos(angle)
            estimated_y = yc + radius * sin(angle)
            xy_estimate = estimated_x, estimated_y

            # steps = 1
            # while True:
            #     # check how many steps it will take to get there for Hunter
            #     if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
            #         break
            #     steps += 1
            #
            #     totalAngle += rotationSign * turning
            #     estimated_x = xc + radius * cos(totalAngle)
            #     estimated_y = yc + radius * sin(totalAngle)
            #     xy_estimate = estimated_x, estimated_y


    coords.append(target_measurement)

    OTHER = (distances, angles, coords, xy_estimate, steps, xy_pf, turnAngle, x, y)
    turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading) # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)

    return turning, distance, OTHER


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



def createParticles(worldX, worldY, turning, distance, N):
    # create new particles
    for i in range(N):
        r = robot(random.uniform(worldX - 25, worldX + 25),
                  random.uniform(worldY - 25, worldY + 25),
                  random.random() * 2.0 * pi, # noise in orientation
                  turning = turning,
                  distance = distance)
        r.set_noise(new_t_noise = 2.,
                    new_d_noise = 2.,
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
def senseToLandmarks(targetX, targetY):
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


def particle_filter(targetMeasurementToLandmarks, turning, distance, x, y):
    global particles

    # add some fresh particles before resampling
    if x is not None:
        createParticles(x, y, turning, distance, 3)

    # PREDICT by moving
    p2 = []
    for i in range(len(particles)):
        newParticle = move(particles[i].x, particles[i].y, turning, distance, particles[i].heading, particles[i].distance_noise, particles[i].turning_noise, particles[i].measurement_noise)
        p2.append(newParticle)
    particles = p2

    # UPDATE by creating weights
    w = []
    for i in range(len(particles)):
        particle = particles[i]
        mp = calculateWeight( particle.x, particle.y, targetMeasurementToLandmarks)
        w.append(  mp )


    # RESAMPLING
    p3 = []
    index = int(random.random() * len(particles))
    beta = 0.0
    mw = max(w)
    for i in range(len(particles)):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % len(particles)
        p3.append( particles[index] )
    particles = p3
    # end resampling

    return get_position(particles)


def demo_grading_visual(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization

    turtle.setup(800, 800)

    window = turtle.Screen()
    window.bgcolor('white')
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.2, 0.2, 0.2)
    prediction = turtle.Turtle()
    prediction.shape('circle')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    #prediction.penup()
    #broken_robot.penup()
    #End of Visualization

    handle = 0.
    prediction_handle = 0.
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # if ctr >= 43:
        #     #time.sleep(0.5)
        #     prediction.color('red')
        #     broken_robot.color('black')

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        #print "actual target position", target_position
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        # predictedPosition = OTHER[5]
        # print "predicted distance to target: ", distance_between(predictedPosition, target_position)
        # print "measured distance to target:  ", distance_between(target_measurement, target_position)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        #print ctr + 1

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        broken_robot.clearstamp(handle)
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        handle = broken_robot.stamp()
        prediction.clearstamp(prediction_handle)
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
        prediction_handle = prediction.stamp()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."



    return caught

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):

    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
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
            return 1000
    return caught


demo_grading_visual(hunter, target, next_move_straight_line)
#demo_grading(hunter, target, next_move_straight_line)

# scores = []
# fails = 0
# for i in range(100):
#     print i
#     particles = []
#     target = robot(0.0, 10.0, 0.0, -2*pi / 30, 1.5)
#     target.set_noise(0.0, 0.0, .05*target.distance)
#     hunter = robot(-10.0, -10.0, 0.0)
#     score = demo_grading(hunter, target, next_move_straight_line)
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





