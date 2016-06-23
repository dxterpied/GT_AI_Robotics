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


# this function sometimes works; does not work most of the time
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = None
        steps = 0
    else:
        distances, angles, coords, xy_estimate, steps = OTHER

        if len(coords) == 1:
            x1, y1 = coords[0]
            x2, y2 = target_measurement
            hypotenuse1 = distance_between(coords[0], target_measurement)
            y1Delta = y2 - y1
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

            angles.append(abs(predictedTurnAngleAvg))
            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            if xy_estimate is None:
                steps = 1
                xy_estimate = point3[0], point3[1]
                while True:
                    newR = robot(xy_estimate[0], xy_estimate[1], headingAngle2, avgAngle, avgDT)
                    newR.move_in_circle()
                    xy_estimate = newR.x, newR.y
                    headingAngle2 = newR.heading
                    distanceBetweenHunterAndRobot = distance_between(hunter_position, xy_estimate)
                    # check how many steps it will take to get there for Hunter
                    projectedDistance = steps * max_distance

                    #print "xy_estimate", xy_estimate

                    if projectedDistance >= distanceBetweenHunterAndRobot:
                        #print xy_estimate, steps
                        targetSet = True
                        break

                    steps += 1
                    if steps > 50:
                        break
            else:
                steps -= 1
                #print "decrement steps", steps
                if steps == 0:
                    xy_estimate = None

    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps)

    if xy_estimate is None:
        xy_estimate = target_measurement
    heading_to_target = get_heading(hunter_position, xy_estimate)
    turning = heading_to_target - hunter_heading # turn towards the target

    #print hunter_position, xy_estimate, heading_to_target, hunter_heading, heading_difference
    distance = distance_between(hunter_position, xy_estimate)

    #print turning, distance
    return turning, distance, OTHER



def next_move_old(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""

    #time.sleep(0.5)
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
    else:
        distances, angles, coords = OTHER

        if len(coords) == 1:
            x1, y1 = coords[0]
            x2, y2 = target_measurement
            hypotenuse1 = distance_between(coords[0], target_measurement)
            y1Delta = y2 - y1
            headingAngle1 = asin(y1Delta / hypotenuse1)
            #angles.append(headingAngle1)
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

            angles.append(abs(predictedTurnAngleAvg))
            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            newR = robot(point3[0], point3[1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            newR = robot(newR.x, newR.y, headingAngle2, avgAngle, avgDT)
            # move again to try to predict where it will be in one step
            newR.move_in_circle()
            newR = robot(newR.x, newR.y, headingAngle2, avgAngle, avgDT)
            # move again to try to predict where it will be in one step
            newR.move_in_circle()
            newR = robot(newR.x, newR.y, headingAngle2, avgAngle, avgDT)
            # move again to try to predict where it will be in one step
            newR.move_in_circle()
            newR = robot(newR.x, newR.y, headingAngle2, avgAngle, avgDT)
            # move again to try to predict where it will be in one step
            newR.move_in_circle()

            xy_estimate = newR.x, newR.y

    coords.append(target_measurement)
    OTHER = (distances, angles, coords)

    if xy_estimate is None:
        xy_estimate = target_measurement
    heading_to_target = get_heading(hunter_position, xy_estimate)
    heading_difference = heading_to_target - hunter_heading

    #print hunter_position, xy_estimate, heading_to_target, hunter_heading, heading_difference
    turning = heading_difference # turn towards the target
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
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 15.0  #change Size of animation
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
        #print target_position
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

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

demo_grading(hunter, target, next_move)
#demo_grading_visual(hunter, target, next_move)





