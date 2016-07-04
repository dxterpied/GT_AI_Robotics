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

maxX = 0
maxY = 0
minX = 0
minY = 0


deltaT = 0.1
x = matrix([[0.],
            [0.],
            [0.],
            [0.]]) # initial state
u = matrix([[0.], [0.], [0.], [0.]]) # external motion

F = matrix([
        [1., 0., 0.0, 0.], # this is to update x = 1 * x + 0 * y + deltaT * x_prime + 0 * y_prime = x + deltaT*x_prime
        [0., 1., 0., 0.0], # this is to update y = 0 * x + 1 * y + 0 * x_prime + deltaT * y_prime = y + deltaT * y_prime
        [0., 0., 1., 0.], # this is to update x_prime = 0 * x + 0 * y + 1 * x_prime + 0 * y_prime = x_prime
        [0., 0., 0., 1.]  # this is to update y_prime = 0 * x + 0 * y + 0 * x_prime + 1 * y_prime = y_prime
    ]) # next state function: generalize the 2d version to 4d

H = matrix([ [1., 0., 0., 0.],
            [0., 1., 0., 0.]]) # measurement function: reflect the fact that we observe x and y but not the two velocities
R = matrix([
    [0.1, 0.],
    [0., 0.1]]) # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
I = matrix([ [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]  ]) # 4d identity matrix

#stats for this method:
# average score:  155
# minimum score:  22
# maximum score:  985
# fails:  7

def next_move_cut_angle(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):



    #time.sleep(0.5)
    xy_estimate = target_measurement
    xy_estimate_first_step = target_measurement
    xy_estimate_second_step = target_measurement

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
            xy_estimate_first_step = newR.x, newR.y
            newR.move_in_circle()
            xy_estimate_second_step = newR.x, newR.y
            newR.move_in_circle()

            xy_estimate = newR.x, newR.y

    coords.append(target_measurement)
    OTHER = (distances, angles, coords)

    distance = distance_between(hunter_position, xy_estimate)
    distance_first_step = distance_between(hunter_position, xy_estimate_first_step)
    distance_second_step = distance_between(hunter_position, xy_estimate_second_step)

    # if first step prediction is within hunter's reach, go there
    if distance_first_step < max_distance:
        distance = distance_first_step
        xy_estimate = xy_estimate_first_step

    # if second step prediction is within hunter's reach, go there
    if distance_second_step < max_distance:
        distance = distance_second_step
        xy_estimate = xy_estimate_second_step

    heading_to_target = get_heading(hunter_position, xy_estimate)
    turning = heading_to_target - hunter_heading # turn towards the target


    # if heading_to_target < -2.0 and hunter_heading > 0:
    #     print heading_to_target, hunter_heading, turning
        #time.sleep(2)

    if abs(turning) > pi:
        turning = turning % pi



    return turning, distance, OTHER



# this is much much much worse than the regular next_move() ......
def next_move_KF(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    predictedPosition = (0, 0)
    xy_estimate = None
    steps = 0

    if OTHER is None:
        P_matrix = matrix([
            [0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 1000., 0.],
            [0., 0., 0., 1000.] ]) # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
        measurements = [(0.0, 0.0)]
        prevAngle = 0.
    else:
        measurements, P_matrix, prevAngle, xy_estimate, steps = OTHER

    print "faulty target", target_measurement
    x = matrix([[target_measurement[0]], [target_measurement[1]], [0.], [0.]])
    new_x, P_matrix = kalman_filter(x, P_matrix, measurements[-1:]) # take last n items in measurements
    adjustedTarget = (new_x.value[0][0], new_x.value[1][0]) # get new x and y from new_x matrix
    print "adjusted target", adjustedTarget

    prevCoord = measurements[len(measurements) - 1]
    x1Delta = adjustedTarget[0] - prevCoord[0]
    y1Delta = adjustedTarget[1] - prevCoord[1]
    heading = atan2(y1Delta, x1Delta)
    turning = heading - prevAngle
    distance = distance_between(prevCoord, adjustedTarget)

    newR = robot(adjustedTarget[0], adjustedTarget[1], heading, turning, distance)
    newR.move_in_circle()
    predictedPosition = newR.x, newR.y

    if xy_estimate is None:

        broken_robot = turtle.Turtle()
        broken_robot.shape('turtle')
        broken_robot.color('red')
        #broken_robot.resizemode('user')
        broken_robot.shapesize(0.2, 0.2, 0.2)

        steps = 1

        while True:
            #time.sleep(0.1)
            xy_estimate = newR.x, newR.y
            headingAngle2 = newR.heading
            distanceBetweenHunterAndRobot = distance_between(hunter_position, xy_estimate)
            # check how many steps it will take to get there for Hunter
            projectedDistance = steps * max_distance

            broken_robot.setheading(headingAngle2 * 180/pi)
            broken_robot.goto(newR.x * 25, newR.y * 25 - 200)
            broken_robot.stamp()

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
        if steps == 0:
            xy_estimate = None

    prevAngle = heading
    measurements.append(adjustedTarget)
    OTHER = (measurements, P_matrix, prevAngle, xy_estimate, steps)

    if xy_estimate is None:
        xy_estimate = target_measurement

    heading_to_target = get_heading(hunter_position, xy_estimate)
    heading_to_target2 = get_heading(hunter_position, predictedPosition)

    turning = heading_to_target - hunter_heading # turn towards the target
    turning2 = heading_to_target2 - hunter_heading # turn towards the target

    #print hunter_position, xy_estimate, heading_to_target, hunter_heading, heading_difference
    distance = distance_between(hunter_position, xy_estimate)
    distance2 = distance_between(hunter_position, predictedPosition)

    # if distance2 < distance:
    #      turning = turning2
    #      distance = distance2

    # if steps == 1 and distance2 < distance:
    #      turning = turning2
    #      distance = distance2

    # turning = turning2
    # distance = distance2

    #print turning, distance
    return turning, distance, OTHER

#Stats:
# average score:  165
# minimum score:  21
# maximum score:  991
# fails:  6
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
            x1, y1 = coords[0]
            x2, y2 = target_measurement
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

            newR = robot(point3[0], point3[1], headingAngle2, avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y

            if xy_estimate is None:

                # broken_robot = turtle.Turtle()
                # broken_robot.shape('turtle')
                # broken_robot.color('red')
                # #broken_robot.resizemode('user')
                # broken_robot.shapesize(0.2, 0.2, 0.2)

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
    OTHER = (distances, angles, coords, xy_estimate, steps)


    if xy_estimate is None:
        xy_estimate = target_measurement

    heading_to_target = get_heading(hunter_position, xy_estimate)
    heading_to_target2 = get_heading(hunter_position, predictedPosition)

    turning = heading_to_target - hunter_heading # turn towards the target
    if abs(turning) > pi:
        turning = turning % pi

    turning2 = heading_to_target2 - hunter_heading # turn towards the target

    # if heading_to_target < -2.0 and hunter_heading > 0:
    #     print heading_to_target, hunter_heading, turning
    #     time.sleep(2)


    #print hunter_position, xy_estimate, heading_to_target, hunter_heading, turning
    distance = distance_between(hunter_position, xy_estimate)
    distance2 = distance_between(hunter_position, predictedPosition)

    if distance2 <= max_distance:
        turning = turning2
        distance = distance2
        OTHER = (distances, angles, coords, None, steps)


    # if steps == 1 and distance2 < distance:
    #      turning = turning2
    #      distance = distance2

    #turning = turning2
    #distance = distance2


    #print turning, distance
    return turning, distance, OTHER



def next_move_old(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):


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
    global maxX
    global maxY
    global minX
    global minY

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

        if target_position[0] > maxX:
            maxX = target_position[0]
        if target_position[1] > maxY:
            maxY = target_position[1]
        if target_position[0] < minX:
            minX = target_position[0]
        if target_position[1] < minY:
            minY = target_position[1]


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

def kalman_filter(x, P, measurements):
    for n in range(len(measurements)):

        # PREDICTION  (based on theory). Uses total probability and convolution
        x = (F * x) + u              # in Michel van Biezen it's x1 = F * x0 + B * u1 + w1: https://www.youtube.com/watch?v=mRf-cL2mjo4
        P = F * P * F.transpose() # + Q  the Q matrix (process noise) is not present here

        # MEASUREMENT UPDATE
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)  # Innovation or measurement residual
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse() # Kalman gain
        x = x + (K * y)
        P = (I - (K * H)) * P


    return x,P


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


#demo_grading(hunter, target, next_move_straight_line)
demo_grading_visual(hunter, target, next_move_cut_angle)

#print maxX, maxY, minX, minY

# scores = []
# fails = 0
# for i in range(10000):
#     target = robot(0.0, 0.0, 0.0, 2*pi / 30, 1.5)
#     target.set_noise(0.0, 0.0, measurement_noise)
#     hunter = robot(-10.0, -20.0, 0.0)
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






