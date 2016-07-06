
from robot import *
from math import *
from matrix import *
import random
import turtle
import time


# it appears 4 landmarks is optimal; decreasing landmarks degrades performance; increasing does not seem to have any positive impact
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
size_multiplier= 15.0  #change Size of animation
N = 1000
measurement_noise = 1.0
particles = []

bumblebee = turtle.Turtle()
bumblebee.shape('square')
bumblebee.color('yellow')
bumblebee.shapesize(0.2, 0.2, 0.2)


def next_move_straight_line(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    predictedPosition = [0, 0]
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
    else:
        distances, angles, coords, xy_estimate, steps = OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
            distances.append(hypotenuse1)
            xy_estimate = target_measurement

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

            # create particles only after approximate turning and distance are known
            if len(particles) == 0:
                createParticles(target_measurement[0], target_measurement[1], avgAngle, avgDT)

            Z = senseToLandmarks(target_measurement[0], target_measurement[1])
            x_y = particle_filter(Z, avgAngle, avgDT)

            xy_estimate = x_y

            newR = robot(x_y[0], x_y[1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y

            bumblebee.goto(xy_estimate[0] * size_multiplier, xy_estimate[1] * size_multiplier - 200)
            bumblebee.stamp()


            # if xy_estimate is None:
            #     steps = 1
            #     xy_estimate = predictedPosition
            #
            #     broken_robot = turtle.Turtle()
            #     broken_robot.shape('turtle')
            #     broken_robot.color('red')
            #     broken_robot.shapesize(0.2, 0.2, 0.2)
            #
            #
            #     while True:
            #         time.sleep(0.5)
            #         distanceBetweenHunterAndRobot = distance_between(hunter_position, xy_estimate)
            #         # check how many steps it will take to get there for Hunter
            #         projectedDistance = steps * max_distance
            #
            #         broken_robot.goto(newR.x * size_multiplier, newR.y * size_multiplier - 200)
            #         broken_robot.stamp()
            #
            #         if projectedDistance >= distanceBetweenHunterAndRobot:
            #             #print xy_estimate, steps
            #             break
            #
            #         steps += 1
            #         if steps > 50:
            #             break
            #         # move target one more step
            #         newR.move_in_circle()
            #         xy_estimate = newR.x, newR.y
            #
            # else:
            #     steps -= 1
            #     #print "decrement steps", steps
            #     if steps <= 0:
            #         xy_estimate = None

    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps)

    if xy_estimate is None:
        xy_estimate = target_measurement

    heading_to_target = get_heading(hunter_position, xy_estimate)
    turning = angle_trunc(heading_to_target - hunter_heading) # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)

    return turning, distance, OTHER


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


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


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

    #turtle.setup(800, 800)

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
        #print "actual target position", target_position
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

        #print ctr + 1

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
            return 1000



    return caught

target = robot(0.0, 15.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -5.0, 0.0)

demo_grading_visual(hunter, target, next_move_straight_line)
#demo_grading(estimate_next_pos, test_target)

#turtle.getscreen()._root.mainloop()





