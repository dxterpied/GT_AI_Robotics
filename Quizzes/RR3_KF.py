
from robot import *
from math import *
from matrix import *
import random
import time
import numpy

# this Kalman filter works amazingly well here with the default measurement noise:

# 10,000 runs:
# average score:  39.5241
# minimum score:  14
# maximum score:  379
# fails:  0


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



# These are the equations used for prediction:
#  x(n+1) = x(n) + distance * cos(self.heading)
#  y(n+1) = y(n) + distance * sin(self.heading)

# state transition matrix
F = numpy.matrix([
    [1., 0.],
    [0., 1.]
    ])

H = numpy.matrix([[1., 0.], [0., 1.]])
x = numpy.matrix([[0.], # x
                  [0.]  # y
                  ]) # initial state vector
P = numpy.matrix([[1000., 0.    ],
                  [0.,     1000.]])
R = numpy.matrix([[1., 0.], [0. , 1.]]) # measurement uncertainty
B = numpy.matrix([[1., 0.], [0. , 1.]]) # control matrix
I = numpy.matrix([[1., 0.], [0. , 1.]]) # identity


kf = KalmanFilter(F, H, x, P, R, I)

def next_move_KF(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    xy_estimate = target_measurement

    if OTHER is None:
        distances = []
        angles = []
        coords = []
    else:
        distances, angles, coords = OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
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

            angles.append(predictedTurnAngleAvg)
            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            Z = numpy.matrix([
                        [ target_measurement[0] ],
                        [target_measurement[1] ]])
            u = numpy.matrix([[avgDT * cos(headingAngle2), 0.],
                              [0., avgDT * sin(headingAngle2)]]) # control matrix
            newState = kf.filter(Z, u)


            newR = robot(newState[0,0], newState[1,1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y

            # replace target_measurement with estimated measurement to be used later for average calculations
            # this technique greatly improved the scor
            target_measurement = (newState.item(0), newState.item(3))


    coords.append(target_measurement)


    OTHER = (distances, angles, coords)

    heading_to_target = angle_trunc(get_heading(hunter_position, xy_estimate))
    heading_difference = heading_to_target - hunter_heading
    turning = angle_trunc(heading_difference) # turn towards the target
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
            return 1000
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


target = robot(0.0, 0.0, 0.0, -2*pi / 30, 1.5)
measurement_noise = .05 * target.distance
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -20.0, 0.0)

#demo_grading(hunter, target, next_move_KF)
#demo_grading_visual(hunter, target, next_move_KF)

scores = []
fails = 0
for i in range(1000):
    print i
    target = robot(0.0, 0.0, 0.0, -2*pi / 30, 1.5)
    measurement_noise = .05 * target.distance
    target.set_noise(0.0, 0.0, measurement_noise)
    hunter = robot(-10.0, -20.0, 0.0)
    kf = KalmanFilter(F, H, x, P, R, I)

    score = demo_grading(hunter, target, next_move_KF)

    if score == 1000:
        fails += 1
    else:
        scores.append(score)

print "average score: ", sum(scores)/ float(len(scores))
print "minimum score: ", min(scores)
print "maximum score: ", max(scores)
print "fails: ", fails


# 1000 runs with positive angle (counterclockwise):
# average score:  159.391173521
# minimum score:  14
# maximum score:  914
# fails:  3

# 1000 runs with negative angle (clockwise):
# average score:  130.364
# minimum score:  8
# maximum score:  928
# fails:  0