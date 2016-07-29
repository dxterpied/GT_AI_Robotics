# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random
import time
import turtle    #You need to run this locally to use the turtle module

# Testing parameters:
#
# -20.0 <= target_starting_position_x <= 20.0
# -20.0 <= target_starting_position_y <= 20.0
# 0 <= target_starting_heading < 2*math.pi
# 10 <= target_period <= 50 (target might go either clockwise or counterclockwise)
# 1 <= target_speed <= 5


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


# average score:  235.333333333
# minimum score:  22
# maximum score:  996
# fails:  16
# this approach is taken from RR4_EKF. Works worse than my straight_line functions
def next_move_straight_line_Worse(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    predictedPosition = [0, 0]
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        turnAngle = []
    else:
        distances, angles, coords, xy_estimate, steps, turnAngle = OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
            distances.append(hypotenuse1)
            xy_estimate = target_measurement

        elif len(coords) >= 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = target_measurement

            # this is done to determine clock wise or counter clock wise rotation
            turnAngle.append(calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1]))
            rotationSign = getRotationSign(turnAngle)

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

            newR = robot(point3[0], point3[1], headingAngle2, rotationSign * avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y
            xy_estimate = newR.x, newR.y

            # distance from hunter to predicted target position
            dist_to_target = distance_between(predictedPosition, hunter_position)

            print int( dist_to_target / max_distance )
            for d in range( int( dist_to_target / max_distance ) ):
                # look ahead d moves and go that way
                newR.move_in_circle()
                xy_estimate = newR.x, newR.y

    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps, turnAngle)
    if xy_estimate is None:
        xy_estimate = target_measurement

    turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading) # turn towards the target
    turning2 = angle_trunc(get_heading(hunter_position, predictedPosition) - hunter_heading) # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)
    distance2 = distance_between(hunter_position, predictedPosition)

    if distance2 <= max_distance:
        turning = turning2
        distance = distance2
        OTHER = (distances, angles, coords, None, steps, turnAngle)

    return turning, distance, OTHER


# 1000 runs counterclockwise (+):
# average score:  153.075
# minimum score:  21
# maximum score:  854
# fails:  0

# 1000 runs clockwise (-):
# average score:  142.162162162
# minimum score:  12
# maximum score:  907
# fails:  1
def next_move_straight_line(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    predictedPosition = [0, 0]
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        turnAngle = []
    else:
        distances, angles, coords, xy_estimate, steps, turnAngle = OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
            distances.append(hypotenuse1)
            xy_estimate = target_measurement

        elif len(coords) >= 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = target_measurement

            # this is done to determine clock wise or counter clock wise rotation
            turnAngle.append(calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1]))
            rotationSign = getRotationSign(turnAngle)

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

            newR = robot(point3[0], point3[1], headingAngle2, rotationSign * avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y

            if xy_estimate is None:

                # broken_robot = turtle.Turtle()
                # broken_robot.shape('turtle')
                # broken_robot.color('red')
                # #broken_robot.resizemode('user')
                # broken_robot.shapesize(0.2, 0.2, 0.2)

                steps = 1

                # distance from hunter to predicted target position
                # dist_to_target = distance_between(predictedPosition, hunter_position)
                #
                # for i in range( int( dist_to_target / max_distance ) ):
                #     # look ahead d moves and go that way
                #     xy_estimate = newR.x, newR.y
                #     newR.move_in_circle()
                #     steps += 1


                while True:
                    #time.sleep(0.1)
                    xy_estimate = newR.x, newR.y
                    distanceBetweenHunterAndRobot = distance_between(hunter_position, xy_estimate)
                    # check how many steps it will take to get there for Hunter
                    projectedDistance = steps * max_distance

                    # broken_robot.setheading(headingAngle2 * 180/pi)
                    # broken_robot.goto(newR.x * 20, newR.y * 20 - 200)
                    # broken_robot.stamp()

                    if projectedDistance >= distanceBetweenHunterAndRobot:
                        break

                    steps += 1
                    if steps > 50:
                        break

                    newR.move_in_circle()

            else:
                steps -= 1
                if steps <= 0:
                    xy_estimate = None

    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps, turnAngle)
    if xy_estimate is None:
        xy_estimate = target_measurement

    distance2 = distance_between(hunter_position, predictedPosition)

    if distance2 <= max_distance:
        turning = angle_trunc(get_heading(hunter_position, predictedPosition) - hunter_heading)
        distance = distance2
        OTHER = (distances, angles, coords, None, steps, turnAngle)
    else:
        turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading)
        distance = distance_between(hunter_position, xy_estimate)


    return turning, distance, OTHER


# this one calculates straight line on every iteration; that's the modification; it performs better than just straight line approach
# average score:  125.2
# minimum score:  12
# maximum score:  677
# fails:  0
def next_move_straight_line_modified(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    predictedPosition = [0, 0]
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        turnAngle = []
    else:
        distances, angles, coords, xy_estimate, steps, turnAngle = OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
            distances.append(hypotenuse1)
            xy_estimate = target_measurement

        elif len(coords) >= 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = target_measurement

            # this is done to determine clock wise or counter clock wise rotation
            turnAngle.append(calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1]))
            rotationSign = getRotationSign(turnAngle)

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

            newR = robot(point3[0], point3[1], headingAngle2, rotationSign * avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y
            xy_estimate = newR.x, newR.y

            # try to find the shortest straight path from hunter position to predicted target position
            steps = 1
            while True:
                # check if hunter will reach target in a straight line
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1
                newR.move_in_circle()
                xy_estimate = newR.x, newR.y


    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps, turnAngle)
    if xy_estimate is None:
        xy_estimate = target_measurement

    distance2 = distance_between(hunter_position, predictedPosition)

    if distance2 <= max_distance:
        turning = angle_trunc(get_heading(hunter_position, predictedPosition) - hunter_heading)
        distance = distance2
        OTHER = (distances, angles, coords, None, steps, turnAngle)
    else:
        turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading)
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
    size_multiplier= 20.0  #change Size of animation
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

target = robot(0.0, 15.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -5.0, 0.0)


#demo_grading(hunter, target, next_move_straight_line_modified)
demo_grading_visual(hunter, target, next_move_straight_line_modified)


# scores = []
# fails = 0
# for i in range(1000):
#     print i
#     target = robot(0.0, 0.0, 0.0, -2*pi / 30, 1.5)
#     target.set_noise(0.0, 0.0, measurement_noise)
#     hunter = robot(-10.0, -20.0, 0.0)
#     score = demo_grading(hunter, target, next_move_straight_line_modified)
#     if score == 1000:
#         fails += 1
#     else:
#         scores.append(score)
#
# print "average score: ", sum(scores)/ float(len(scores))
# print "minimum score: ", min(scores)
# print "maximum score: ", max(scores)
# print "fails: ", fails








