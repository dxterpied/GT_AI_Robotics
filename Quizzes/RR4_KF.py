from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random
import numpy
import turtle

# This straight KF performs much worse than the running average (RR4.py) or the PF version.

# These are the equations used for prediction:
#  x(n+1) = x(n) + distance * cos(self.heading)
#  y(n+1) = y(n) + distance * sin(self.heading)

# state transition matrix
F = numpy.matrix([
    [1,0], # x
    [0,1]]) # y

H = numpy.matrix([[1., 0.], [0. , 1.]])
x = numpy.matrix([[0.], [0.]]) # initial state vector
P = numpy.matrix([[1000., 0    ],
                  [0,     1000.]])
R = numpy.matrix([[1., 0.], [0. , 1.]]) # measurement uncertainty
B = numpy.matrix([[1., 0.], [0. , 1.]]) # control matrix
I = numpy.matrix([[1., 0.], [0. , 1.]]) # identity




# straight Kalman filter
class KalmanFilter:
  def __init__(self, F, H, x, P, R, I):
    self.F = F
    self.H = H
    self.x = x
    self.P = P
    self.R = R
    self.I = I

  def filter(self, Z, u):
    # Prediction
    self.x = (self.F * self.x) + B * u
    self.P = (self.F * self.P) * numpy.transpose(self.F)

    # Update
    S = self.H * self.P * numpy.transpose(self.H) + self.R
    K = self.P * numpy.transpose(self.H) * numpy.linalg.inv(S) # Kalman gain
    self.x = self.x + K * (Z - self.H * self.x)
    self.P = (I - K * self.H) * self.P
    return self.x


kf = KalmanFilter(F, H, x, P, R, I)
size_multiplier= 20.0  #change Size of animation

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
target.set_noise(0.0, 0.0, .05 * target.distance)
hunter = robot(-10.0, -10.0, 0.0)

# bumblebee = turtle.Turtle()
# bumblebee.shape('square')
# bumblebee.color('yellow')
# bumblebee.shapesize(0.2, 0.2, 0.2)


def next_move_straight_line(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    predictedPosition = [0, 0]
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        xy_pf = (0, 0)
        turnAngle = 0.0
    else:
        distances, angles, coords, xy_estimate, steps, xy_pf, turnAngle = OTHER

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

            Z = numpy.matrix([[target_measurement[0]], [target_measurement[1]]])
            u = numpy.matrix([[avgDT * cos(headingAngle2), 0.],
                              [0., avgDT * sin(headingAngle2) ]]) # control matrix
            newState = kf.filter(Z, u)


            newR = robot(newState.item(0), newState.item(3), headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y

            # replace target_measurement with estimated measurement to be used later for average calculations
            # this technique greatly improved the scor
            target_measurement = (newState.item(0), newState.item(3))

            # bumblebee.goto(xy_pf[0] * size_multiplier, xy_pf[1] * size_multiplier - 200)
            # bumblebee.stamp()


            if xy_estimate is None:
                steps = 1

                while True:
                    #time.sleep(0.1)
                    xy_estimate = newR.x, newR.y
                    headingAngle2 = newR.heading
                    distanceBetweenHunterAndRobot = distance_between(hunter_position, xy_estimate)
                    # check how many steps it will take to get there for Hunter
                    projectedDistance = steps * max_distance

                    # broken_robot.setheading(headingAngle2 * 180/pi)
                    # broken_robot.goto(newR.x * 25, newR.y * 25 - 200)
                    # broken_robot.stamp()

                    if projectedDistance >= distanceBetweenHunterAndRobot:
                        #print xy_estimate, steps
                        break

                    steps += 1
                    if steps > 50:
                        break

                    newR.move_in_circle()

            else:
                steps -= 1
                #print "decrement steps", steps
                if steps <= 0:
                    xy_estimate = None

    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps, xy_pf, turnAngle)
    if xy_estimate is None:
        xy_estimate = target_measurement
    heading_to_target = get_heading(hunter_position, xy_estimate)
    heading_to_target2 = get_heading(hunter_position, predictedPosition)
    turning = angle_trunc(heading_to_target - hunter_heading) # turn towards the target
    # if abs(turning) > pi:
    #     turning = turning % pi
    turning2 = angle_trunc(heading_to_target2 - hunter_heading) # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)
    distance2 = distance_between(hunter_position, predictedPosition)
    # if distance to the next predicted step is less than max distance, jump there
    if distance2 <= max_distance:
        turning = turning2
        distance = distance2
        OTHER = (distances, angles, coords, None, steps, xy_pf, turnAngle)

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


# returns the arithmetic means of x, y and orientation. It is already weighted.
def get_position(p):
    x = y = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y


    return [ x/len(p), y/len(p) ]



"""Computes distance between point1 and point2. Points are (x, y) pairs."""
def distance_between(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


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
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    #prediction.penup()
    broken_robot.penup()
    #End of Visualization

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
# for i in range(1000):
#     print i
#     particles = []
#     target = robot(0.0, 10.0, 0.0, 2 * pi / 30, 1.5)
#     target.set_noise(0.0, 0.0, .05 * target.distance)
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

# average score:  463.504896627
# minimum score:  21
# maximum score:  996
# fails:  81


#turtle.getscreen()._root.mainloop()





