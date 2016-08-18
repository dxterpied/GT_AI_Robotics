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
# Employs the strategy of taking the predicted first heading, stepping back and then moving predicted heading in equal increments
# The biggest issue is predicting the initial heading. Even with a perfect prediction of turning, a good prediction of initial heading is required

# with steps_allowed = 60. and measurements_to_pass = 340
#fails:  885
#fails:  860

# with steps_allowed = 20. and measurements_to_pass = 340
# fails:  870
# fails:  855

# with steps_allowed = 40. and measurements_to_pass = 340
# fails:  858
# fails:  885
# fails:  863
# fails:  868
# fails:  896
# fails:  893

size_multiplier = 20.
target = robot(0.0, 5.0, 0.0, 2*pi / 30, 1.5)
target.set_noise(0.0, 0.0, 2.0 * target.distance)
hunter = robot(-10.0, -15.0, 0.0)

turtle.setup(450, 450, 0, 600)
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

# xc = -0.75
# yc = 12.1357733407
# xc = -0.75
# yc = 12.1357733407
# radius = 7.17507917513
#
# xDelta = target.x - xc
# yDelta = target.y - yc
# actualFirstHeading = atan2(yDelta, xDelta)
# estimated_x = xc + radius * cos(actualFirstHeading)
# estimated_y = yc + radius * sin(actualFirstHeading)
# print "target.x", target.x, "target.y", target.y, "estimated_x", round(estimated_x), "estimated_y", round(estimated_y)
#
# print "actual first heading: ",  actualFirstHeading
# print "-------------------------------------------"
# exit()

# actual_initial_heading.goto(target_x * size_multiplier, target_y * size_multiplier - 200)
# actual_initial_heading.stamp()
# actual_center.goto(-0.75 * size_multiplier, 12.1357733407 * size_multiplier - 200)
# actual_center.stamp()

step_size = 0. # increment size in increasing heading prediction
steps_allowed = 40. # number of steps for an increment in the starting heading increase
measurements_to_pass = 340. # number of measurements to pass before calculating initial heading

# actual center: -0.75, 12.1357733407
# actual radius: 7.17507917513
# actual first heading:  -1.46607657168
# actual turning: 0.209

first_headings = []

actual_xy = (target.x, target.y)

predicted_x = []
predicted_y = []
measured_x = []
measured_y = []

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
    global first_headings

    # get the very first heading angle (measured).
    xDelta = measurements[0][0] - xc
    yDelta = measurements[0][1] - yc
    prevHeading = atan2(yDelta, xDelta)
    firstHeading = prevHeading
    totalAngle = 0.
    startingHeading = 0.
    total_angles = []
    total_angles.append(firstHeading)

    # calculate the turning angle
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

    # calculate first heading
    average_angles = []
    average_angles.append(firstHeading)
    first_headings.append(firstHeading)

    for i in range(1, int( len(measurements) / number_of_steps )):
        index = int(number_of_steps * i) - 1
        average_angles.append( mean([ total_angles[index] ]) )

    startingHeading = mean(average_angles)

    first_headings.append(startingHeading)
    # smooth averages over a little bit; without smoothing the prediction is worse
    startingHeading = mean(first_headings)

    # predicted starting heading seems to be more accurate than the measured one
    # predicted distance to actual:  5.08044473428
    # measured  distance to actual:  5.90970815051
    # using first measured heading as the start
    #startingHeading = firstHeading # actual heading angle: -1.46607657168

    #print "startingHeading", startingHeading # actual first heading:  -1.46607657168

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
    global predicted_x, predicted_y, measured_x, measured_y

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
                #print "startingHeading", startingHeading
                #print "turning", turning # actual turning: 0.209

                # step back one turning distance
                startingHeading = startingHeading - turning
                # calculated the increment size
                step_size = abs( (2 * turning) / ((1000 - measurements_to_pass)/steps_allowed) )


                estimated_x = xc + radius * cos(startingHeading)
                estimated_y = yc + radius * sin(startingHeading)
                xDelta = measurements[0][0] - xc
                yDelta = measurements[0][1] - yc
                measured_heading = atan2(yDelta, xDelta)
                m_x = xc + radius * cos(measured_heading)
                m_y = yc + radius * sin(measured_heading)
                predicted_x.append(estimated_x)
                predicted_y.append(estimated_y)
                measured_x.append(m_x)
                measured_y.append(m_y)
            #
            #     # predicted_initial_heading.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
            elif (len(measurements) % steps_allowed) == 0:
                startingHeading += step_size
                # print "startingHeading", startingHeading

            # estimated_x = xc + radius * cos(startingHeading)
            # estimated_y = yc + radius * sin(startingHeading)
            # predicted_initial_heading.clearstamp(predicted_initial_heading_handle)
            # predicted_initial_heading.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
            # predicted_initial_heading_handle = predicted_initial_heading.stamp()

            # get estimated position
            totalAngle = getHeading(turning, startingHeading, measurements)
            estimated_x = xc + radius * cos(totalAngle)
            estimated_y = yc + radius * sin(totalAngle)
            xy_estimate = estimated_x, estimated_y

            # # estimated center drawing
            # bumblebee.clearstamp(bumblebee_handle)
            # bumblebee.goto(xc * size_multiplier, yc * size_multiplier - 200)
            # bumblebee_handle = bumblebee.stamp()
            # # estimated position drawing
            # hunterbee.clearstamp(hunterbee_handle)
            # hunterbee.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
            # hunterbee_handle = hunterbee.stamp()

            #try to find the shortest straight path from hunter position to predicted target position
            steps = 0
            while True:
                if (steps * max_distance) > distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                totalAngle += angle_trunc(rotationSign * turning)
                estimated_x = xc + radius * cos(totalAngle)
                estimated_y = yc + radius * sin(totalAngle)
                xy_estimate = estimated_x, estimated_y
                steps += 1

            #time.sleep(.5)

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

    window = turtle.Screen()
    window.bgcolor('white')

    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
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

        if ctr >= 300:
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


# demo_grading_visual(hunter, target, next_move_straight_line)
#demo_grading(hunter, target, next_move_straight_line)

scores = []
fails = 0
for i in range(1000):
    print i,
    first_headings = []
    target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
    target.set_noise(0.0, 0.0, 2.0 * target.distance)
    hunter = robot(-10.0, -10.0, 0.0)
    score = demo_grading(hunter, target, next_move_straight_line)
    if score == 1000:
        fails += 1
    else:
        scores.append(score)

print "fails: ", fails


print "predicted distance to actual: ", distance_between(actual_xy, (mean(predicted_x), mean(predicted_y)) )
print "measured  distance to actual: ", distance_between(actual_xy, (mean(measured_x), mean(measured_y)) )


#turtle.getscreen()._root.mainloop()




