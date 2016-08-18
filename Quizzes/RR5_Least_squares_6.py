import matplotlib
matplotlib.use('TkAgg')
import turtle
from robot import *
from numpy import *
import random
import time
from scipy import optimize

# Ilya:
# this one uses circular regression.
# This is an extension of RR5_Least_squares_5.py
# Employs the strategy of moving predicted heading in equal increments
# The biggest issue is predicting the initial heading. Even with a perfect prediction of turning, a good prediction of initial heading is required

# with steps_allowed = 15. and measurements_to_pass = 340
# average score:  782.99378882
# minimum score:  412
# maximum score:  990
# fails:  678


# with steps_allowed = 30. and measurements_to_pass = 330
# average score:  843.515243902
# minimum score:  556
# maximum score:  994
# fails:  672


# with steps_allowed = 30. and measurements_to_pass = 300
# average score:  949.126984127
# minimum score:  804
# maximum score:  999
# fails:  937


# with steps_allowed = 20. and measurements_to_pass = 300
# average score:  851.85620915
# minimum score:  311
# maximum score:  999
# fails:  694


# with steps_allowed = 20. and measurements_to_pass = 340
# average score:  799.724340176
# minimum score:  23
# maximum score:  998
# fails:  659

# with steps_allowed = 15. and measurements_to_pass = 300
# average score:  854.694915254
# minimum score:  369
# maximum score:  999
# fails:  705


# with steps_allowed = 10. and measurements_to_pass = 330
# average score:  846.25
# minimum score:  510
# maximum score:  998
# fails:  696

size_multiplier = 20.
target = robot(0.0, 5.0, 0.0, 2*pi / 30, 1.5)
target.set_noise(0.0, 0.0, 2.0 * target.distance)
hunter = robot(-10.0, -15.0, 0.0)

bumblebee = turtle.Turtle()
bumblebee.shape('square')
bumblebee.color('black')
bumblebee.penup()
bumblebee.shapesize(0.4, 0.4, 0.4)

hunterbee = turtle.Turtle()
hunterbee.shape('turtle')
hunterbee.color('brown')
hunterbee.shapesize(0.3, 0.3, 0.3)
#hunterbee.penup()
bumblebee_handle = 0.
hunterbee_handle = 0.

predicted_initial_heading = turtle.Turtle()
predicted_initial_heading.shape('circle')
predicted_initial_heading.color('orange')
#predicted_initial_heading.penup()
predicted_initial_heading.shapesize(0.4, 0.4, 0.4)

actual_initial_heading = turtle.Turtle()
actual_initial_heading.shape('circle')
actual_initial_heading.color('purple')
actual_initial_heading.penup()
actual_initial_heading.shapesize(0.4, 0.4, 0.4)

actual_center = turtle.Turtle()
actual_center.shape('circle')
actual_center.color('black')
actual_center.penup()
actual_center.shapesize(0.3, 0.3, 0.3)


predicted_initial_heading_handle = 0.

target_x = target.x
target_y = target.y


xDelta = target.x - 0.75
yDelta = target.y - 12.1357733407
actualFirstHeading = atan2(yDelta, xDelta)
estimated_x = -0.75 + 7.17507917513 * cos(actualFirstHeading)
estimated_y = 12.1357733407 + 7.17507917513 * sin(actualFirstHeading)

print "target.x", target.x, "estimated_x", estimated_x

# print "actual first heading: ",  actualFirstHeading
# print "-------------------------------------------"

actual_initial_heading.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
actual_initial_heading.stamp()

actual_center.goto(-0.75 * size_multiplier, 12.1357733407 * size_multiplier - 200)
actual_center.stamp()


test_measurements = [ [], [], [], [] ]
first_headings = []

step_size = 0. # increment size in increasing heading prediction
steps_allowed = 50. # number of steps for an increment in the starting heading increase
measurements_to_pass = 300. # number of measurements to pass before calculating initial heading

# actual center: -0.75, 12.1357733407
# actual radius: 7.17507917513
# actual first heading:  -1.67551608191
# actual turning: 0.209

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def least_squares(x, y):

    x = r_[x]
    y = r_[y]

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

    return radius, xc, yc


def getTurningAndHeading(measurements, rotationSign, radius, xc, yc):
    global predicted_initial_heading_handle, first_headings

    # get the very first heading angle (measured).
    xDelta = measurements[0][0] - xc
    yDelta = measurements[0][1] - yc
    # xDelta = -0.75 - xc
    # yDelta = 12.1357733407 - yc
    prevHeading = atan2(yDelta, xDelta)
    firstHeading = prevHeading
    totalAngle = 0.
    startingHeading = 0.
    total_angles = []
    total_angles.append(firstHeading)

    for coords in measurements[1:]:
        x, y = coords
        # get heading to measurement
        xDelta = x - xc
        yDelta = y - yc
        currentHeading = atan2(yDelta, xDelta)

        total_angles.append(angle_trunc(currentHeading))

        # difference between current and previous
        if currentHeading < 0. and abs(currentHeading) > pi/2 and prevHeading > 0. and prevHeading > pi/2:
            turningAngle = 2 * pi + currentHeading - prevHeading
        elif currentHeading > 0. and currentHeading > pi/2 and prevHeading < 0. and abs(prevHeading) > pi/2:
            turningAngle = -(2 * pi + currentHeading - prevHeading)
        else:
            turningAngle = currentHeading - prevHeading

        if (turningAngle * rotationSign) > 0: # if signs match, it means rotation in the same direction
            totalAngle += abs(turningAngle)
            prevHeading = currentHeading

    # this is the average turning angle for this round of calculations
    turning = abs(totalAngle / len(measurements))
    number_of_steps = round(2*pi / turning)
    turning = 2*pi / number_of_steps

    # print "turning", turning # angle prediction is sometimes accurate, sometimes not; actual angle is 0.209
    # print "number_of_steps", number_of_steps

    # go through each loop of measurements and calculate average first heading for each loop
    average_angles = []
    average_angles.append(firstHeading)
    first_headings.append(firstHeading)
    for i in range(1, int( len(measurements) / number_of_steps )):
        # take N measurements around the assumed starting heading
        index = int(number_of_steps * i) - 1
        #print "using ", total_angles[index], "index: ", index
        aver = mean([ total_angles[index] ])
        average_angles.append(aver)

    startingHeading = mean(average_angles)

    first_headings.append(startingHeading)
    # smooth averages over a little bit; it makes it only worse.......
    startingHeading = mean(first_headings)

    startingHeading = -1.67551608191 #   firstHeading

    #print "startingHeading", startingHeading # actual first heading:  -1.67551608191

    return turning, startingHeading


# calculate the heading
def getHeading(turning, startingHeading, measurements):
    # calculate the destination angle starting from the first heading
    addedAngle = turning * len(measurements)
    #print average_angle, " * ", len(measurements), " = ", addedAngle
    totalAngle = startingHeading + addedAngle

    return totalAngle


# determine rotation direction by using cross product of three points
# returns negative or positive number depending on the direction of rotation
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
    xy_estimate = target_measurement
    global bumblebee_handle, hunterbee_handle, step_size, steps_allowed, measurements_to_pass, predicted_initial_heading_handle

    if OTHER is None:
        measurements = []
        xy_estimate = target_measurement
        turnAngle = [] # contains all angles from measurements
        x = []
        x.append(target_measurement[0])
        y = []
        y.append(target_measurement[1])
        turning = 0.
        startingHeading = 0.
    else:
        measurements, turnAngle, x, y, turning, startingHeading = OTHER

        # collect measurements
        x.append(target_measurement[0])
        y.append(target_measurement[1])

        if len(measurements) > measurements_to_pass:

            point1 = measurements[len(measurements) - 16]
            point2 = measurements[len(measurements) - 8]
            point3 = target_measurement

            rotationDirection = calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1])
            turnAngle.append(rotationDirection)
            rotationSign = getRotationSign(turnAngle)

            # estimate radius and center using least squares
            radius, xc, yc = least_squares(x, y)
            #print "radius", radius # actual is 7.175; estimate is about 7.5

            radius = 0.9 * radius # least square overestimates radius by about 10%;

            # get estimated turning and total angle traveled from measured start
            if turning == 0. :
                turning, startingHeading = getTurningAndHeading(measurements, rotationSign, radius, xc, yc)
                # print "startingHeading", startingHeading

                estimated_x = xc + radius * cos(startingHeading)
                estimated_y = yc + radius * sin(startingHeading)
                predicted_initial_heading.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)

                step_size = abs( (3 * turning) / ((1000 - measurements_to_pass)/steps_allowed) )
                #startingHeading = startingHeading - turning/2

                # startingHeading = startingHeading - 1.5 * turning
                # estimated_x = xc + radius * cos(startingHeading)
                # estimated_y = yc + radius * sin(startingHeading)
                # predicted_initial_heading.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)

            #print startingHeading

            # elif (len(measurements) % steps_allowed) == 0:
            #     startingHeading += step_size
                # print "startingHeading", startingHeading

            # estimated_x = xc + radius * cos(startingHeading)
            # estimated_y = yc + radius * sin(startingHeading)
            # actual center: -0.75, 12.1357733407
            estimated_x = -0.75 + 7.17507917513 * cos(startingHeading)
            estimated_y = 12.1357733407 + 7.17507917513 * sin(startingHeading)

            predicted_initial_heading.clearstamp(predicted_initial_heading_handle)
            predicted_initial_heading.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
            predicted_initial_heading_handle = predicted_initial_heading.stamp()

            totalAngle = getHeading(turning, startingHeading, measurements)

            # get estimated position
            estimated_x = xc + radius * cos(totalAngle)
            estimated_y = yc + radius * sin(totalAngle)
            xy_estimate = estimated_x, estimated_y

            # bumblebee.clearstamp(bumblebee_handle)
            # bumblebee.goto(xc * size_multiplier, yc * size_multiplier - 200)
            # bumblebee_handle = bumblebee.stamp()
            # hunterbee.clearstamp(hunterbee_handle)
            # hunterbee.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
            # hunterbee_handle = hunterbee.stamp()

            #try to find the shortest straight path from hunter position to predicted target position
            steps = 1
            while True:
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1
                totalAngle += angle_trunc(rotationSign * turning)
                estimated_x = xc + radius * cos(totalAngle)
                estimated_y = yc + radius * sin(totalAngle)
                xy_estimate = estimated_x, estimated_y


    measurements.append(target_measurement)
    OTHER = (measurements, turnAngle, x, y, turning, startingHeading)
    turning = angle_trunc( get_heading(hunter_position, xy_estimate) - hunter_heading ) # turn towards the target
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

    turtle.setup(500, 500)

    window = turtle.Screen()
    window.bgcolor('white')

    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.9, 0.9, 0.9)
    #broken_robot.penup()

    prediction = turtle.Turtle()
    prediction.shape('circle')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    #prediction.penup()

    noise = turtle.Turtle()
    noise.shape('circle')
    noise.color('red')
    noise.resizemode('user')
    noise.shapesize(0.2, 0.2, 0.2)
    #noise.penup()

    prediction_handle = None

    broken_handle = 0.

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

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

        #Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)
        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        broken_robot.clearstamp(broken_handle)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_handle = broken_robot.stamp()

        if prediction_handle is not None:
            prediction.clearstamp(prediction_handle)

        if ctr >= 340:
            prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
            prediction_handle = prediction.stamp()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."


    # print "x_points", x_points
    # print "y_points", y_points
    # print "x_actual", x_actual
    # print "y_actual", y_actual
    return caught

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):

    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.97 is an example. It will change.
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
#     target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
#     target.set_noise(0.0, 0.0, 2.0 * target.distance)
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


#turtle.getscreen()._root.mainloop()




