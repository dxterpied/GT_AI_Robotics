from math import *
import random
import turtle
import time

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def getEuclidianDistance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt( (x2 - x1)**2 + (y2 - y1)**2 )


class robot:

    def __init__(self, x = 0.0, y = 0.0, headingAngle = 0.0, turnAngle = pi/5, distance = 1.0):
        self.x = x
        self.y = y
        self.headingAngle = headingAngle
        self.turnAngle = turnAngle # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0


    def move(self, turnAngle, distance, tolerance = 0.001, max_turning_angle = pi):

        turningAngle = random.gauss(turnAngle, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)
        # truncate to fit physical limitations
        turningAngle = max( - max_turning_angle, turnAngle)
        turningAngle = min( max_turning_angle, turnAngle)
        distance = max(0.0, distance)
        # Execute motion
        self.headingAngle = self.headingAngle + turnAngle
        self.headingAngle = angle_trunc(self.headingAngle)
        self.x += distance * cos(self.headingAngle)
        self.y += distance * sin(self.headingAngle)

        return (self.x, self.y)

    def move_in_circle(self):
        return self.move(self.turnAngle, self.distance)

    def sense(self):
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)


def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    xy_estimate = 0, 0

    if OTHER is None: # this is the first measurement
        OTHER = []
        OTHER.append(measurement)

    else:
        OTHER.append(measurement)

        if len(OTHER) == 3:
            point1 = OTHER[0]
            point2 = OTHER[1]
            point3 = OTHER[2]
            y1Delta = point2[1] - point1[1]
            hyp1 = getEuclidianDistance(point1, point2)
            headingAngle1 = asin(y1Delta / hyp1)

            y2Delta = point3[1] - point2[1]
            hyp2 = getEuclidianDistance(point2, point3)
            headingAngle2 = asin(y2Delta / hyp2)
            predictedTurnAngle = headingAngle2 - headingAngle1
            print "----------------"
            newR = robot(x = point3[0], y = point3[1], headingAngle = headingAngle2, turnAngle = predictedTurnAngle, distance = hyp1)
            print 'newR--->', newR
            newR.move_in_circle()
            xy_estimate = newR.x, newR.y
            print 'ESTIMATE--->', xy_estimate
            print "--------------------"


    return xy_estimate, OTHER


# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 10:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = getEuclidianDistance(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            #print "true position: ", true_position
            #print "prediction: ", position_guess
            localized = True
        if ctr == 10:
            print "Sorry, it took you too many steps to localize the target."
    return localized


target_bot = robot(x=0.0, y=0.0, headingAngle = 0.0, turnAngle = pi/5, distance = 1.0)

ctr = 0
size_multiplier= 25.0  #change Size of animation
# broken_robot = turtle.Turtle()
# broken_robot.resizemode('user')
# broken_robot.shapesize(0.5, 0.5, 0.5)
# broken_robot.penup()

# while ctr <= 4:
#     measurement = target_bot.sense()
#     x, y = target_bot.move_in_circle()
#     #print x, y
#     broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
#     broken_robot.stamp()
#
#     time.sleep(0.5)
#     ctr += 1

demo_grading(estimate_next_pos, target_bot)


