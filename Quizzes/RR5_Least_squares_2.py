import matplotlib
matplotlib.use('TkAgg')
import turtle
from robot import *
from numpy import *
import random

# Ilya:
# this one uses circular regression.
# This is an extension of RR5_Least_squares.py with the new idea of averaging the initial heading angle in getTurnAngle()
# still needs work; not done yet

size_multiplier = 25.
target = robot(0.0, 0.0, 0.0, 2*pi / 30, 1.5)
target.set_noise(0.0, 0.0, 2.0 * target.distance)
hunter = robot(-10.0, -20.0, 0.0)

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
predicted_initial_heading.penup()
predicted_initial_heading.shapesize(0.4, 0.4, 0.4)

actual_initial_heading = turtle.Turtle()
actual_initial_heading.shape('circle')
actual_initial_heading.color('purple')
actual_initial_heading.penup()
actual_initial_heading.shapesize(0.4, 0.4, 0.4)

predicted_initial_heading_handle = 0.

target_x = target.x
target_y = target.y

actual_initial_heading.goto(target.x * size_multiplier, target.y * size_multiplier - 200)
actual_initial_heading.stamp()


test_measurements = [ [], [], [], [] ]

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


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
def getTurnAngle(measurements, rotationSign, radius, xc, yc):
    global predicted_initial_heading_handle
    global test_measurements
    # get the very first heading angle (measured). It's a ball park to get started
    xDelta = measurements[0][0] - xc
    yDelta = measurements[0][1] - yc
    prevHeading = atan2(yDelta, xDelta)
    firstHeading = prevHeading
    totalAngle = 0.
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

    # this is the average turning angle
    average_angle = abs(totalAngle / len(measurements))
    #print "average_angle", average_angle # angle prediction is pretty accurate: ~0.20

    # average the first three headings; this will be the initial heading angle
    #totalAngle = sum(total_angles[:3]) / 3. # initial averaged heading

    numOfPoints = 1
    average_x = 0
    average_y = 0
    for i in range(numOfPoints):
        average_x += measurements[i][0]
        #print average_x
        average_y += measurements[i][1]
    average_x = average_x/numOfPoints
    average_y = average_y/numOfPoints
    test_measurements[0].append((average_x, average_y))


    numOfPoints = 2
    average_x = 0
    average_y = 0
    for i in range(numOfPoints):
        average_x += measurements[i][0]
        #print average_x
        average_y += measurements[i][1]
    average_x = average_x/numOfPoints
    average_y = average_y/numOfPoints
    test_measurements[1].append((average_x, average_y))


    numOfPoints = 3
    average_x = 0
    average_y = 0
    for i in range(numOfPoints):
        average_x += measurements[i][0]
        #print average_x
        average_y += measurements[i][1]
    average_x = average_x/numOfPoints
    average_y = average_y/numOfPoints
    test_measurements[2].append((average_x, average_y))


    numOfPoints = 4
    average_x = 0
    average_y = 0
    for i in range(numOfPoints):
        average_x += measurements[i][0]
        #print average_x
        average_y += measurements[i][1]
    average_x = average_x/numOfPoints
    average_y = average_y/numOfPoints
    test_measurements[3].append((average_x, average_y))


    xcDelta = average_x - xc
    ycDelta = average_y - yc
    totalAngle = atan2(ycDelta, xcDelta)

    estimated_x = xc + radius * cos(totalAngle)
    estimated_y = yc + radius * sin(totalAngle)
    predicted_initial_heading.clearstamp(predicted_initial_heading_handle)
    predicted_initial_heading.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
    predicted_initial_heading_handle = predicted_initial_heading.stamp()

    #print "initial heading", totalAngle

    #print "\tInitial heading", totalAngle, total_angles[:3]

    # calculate the destination angle starting from the first averaged angle
    for i in range(len(measurements)):
        totalAngle += abs(average_angle)

    #print "totalAngle", totalAngle


    #return average_angle, angle_trunc(totalAngle)
    return average_angle, totalAngle


# calculate the average turn angle
def getTurnAngle_1(measurements, rotationSign, xc, yc):
    # get the very first heading angle (measured). It's a ball park to get started
    xDelta = measurements[0][0] - xc
    yDelta = measurements[0][1] - yc
    prevHeading = atan2(yDelta, xDelta)
    firstHeading = prevHeading
    totalAngle = 0.
    total_angles = []

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

        if (turningAngle * rotationSign) > 0: # if signs match, it means rotation in the same direction
            totalAngle += abs(turningAngle)
            prevHeading = currentHeading

    angle = abs(totalAngle / len(measurements))
    #print "angle", angle # angle prediction is pretty accurate: ~0.20

    temp = totalAngle
    totalAngle = firstHeading
    for i in range(len(measurements)):
        totalAngle += abs(angle)
        total_angles.append(totalAngle)

    # average the last three angles
    #print total_angles[len(total_angles) - 4:len(total_angles) - 1]

    totalAngle = sum(total_angles[len(total_angles) - 3:len(total_angles) - 1]) / 2
    #print "total_angles", total_angles
    #print totalAngle

    #exit()

    #print "totalAngle", angle_trunc(totalAngle), angle_trunc(temp)

    return angle, angle_trunc(totalAngle)

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
    global bumblebee_handle, hunterbee_handle


    if OTHER is None:
        measurements = []
        xy_estimate = target_measurement
        turnAngle = [] # contains all angles from measurements
        x = []
        x.append(target_measurement[0])
        y = []
        y.append(target_measurement[1])
    else:
        measurements, turnAngle, x, y = OTHER

        # collect measurements
        x.append(target_measurement[0])
        y.append(target_measurement[1])

        if len(measurements) > 20:

            point1 = measurements[len(measurements) - 16]
            point2 = measurements[len(measurements) - 8]
            point3 = target_measurement

            rotationDirection = calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1])
            turnAngle.append(rotationDirection)
            rotationSign = getRotationSign(turnAngle)

            # estimate radius and center using least squares
            radius, xc, yc = least_squares(x, y)
            # get estimated turning and total angle traveled from measured start
            turning, totalAngle = getTurnAngle(measurements, rotationSign, radius, xc, yc)

            #print "totalAngle", totalAngle

            # get estimated position
            estimated_x = xc + radius * cos(totalAngle)
            estimated_y = yc + radius * sin(totalAngle)
            xy_estimate = estimated_x, estimated_y

            bumblebee.clearstamp(bumblebee_handle)
            bumblebee.goto(xc * size_multiplier, yc * size_multiplier - 200)
            bumblebee_handle = bumblebee.stamp()
            hunterbee.clearstamp(hunterbee_handle)
            hunterbee.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
            hunterbee_handle = hunterbee.stamp()

            #try to find the shortest straight path from hunter position to predicted target position
            steps = 1
            while True:
                # check how many steps it will take to get there for Hunter
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1

                totalAngle += rotationSign * turning
                estimated_x = xc + radius * cos(totalAngle)
                estimated_y = yc + radius * sin(totalAngle)
                xy_estimate = estimated_x, estimated_y


    measurements.append(target_measurement)
    OTHER = (measurements, turnAngle, x, y)
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

    turtle.setup(800, 800)

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
    while not caught and ctr < 100:

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

        # x_points.append(target_measurement[0])
        # y_points.append(target_measurement[1])
        # x_actual.append(target_bot.x)
        # y_actual.append(target_bot.y)


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

        prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
        prediction_handle = prediction.stamp()

        # if ctr > 43:
        #     noise.goto(target_measurement[0] * size_multiplier, target_measurement[1] * size_multiplier - 200)
        #     noise.stamp()
        #     prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
        #     prediction.stamp()



        #print ctr + 1



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
#     particles = []
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


# average score:  441.4
# minimum score:  31
# maximum score:  967
# fails:  975


# x_actual = [0.0, 1.4672214011007085, 2.83753958756461, 4.0510650791270315, 5.054760988665319, 5.80476098866532, 6.268286480227742, 6.4250791751292216, 6.268286480227741, 5.804760988665318, 5.054760988665317, 4.051065079127029, 2.837539587564608, 1.4672214011007072, -8.881784197001252e-16, -1.5000000000000009, -2.9672214011007103, -4.337539587564613, -5.551065079127038, -6.554760988665329, -7.304760988665335, -7.768286480227762, -7.925079175129248, -7.768286480227774, -7.3047609886653575, -6.55476098866536, -5.551065079127074, -4.3375395875646525, -2.9672214011007503, -1.500000000000041, -4.107825191113079e-14]
# y_actual = [15.0, 15.31186753622664, 15.92197250084034, 16.80365037927905, 17.91836761749514, 19.217405723171797, 20.643990497614528, 22.135773340666937, 23.627556183719346, 25.054140958162076, 26.353179063838734, 27.467896302054825, 28.349574180493534, 28.959679145107234, 29.271546681333874, 29.271546681333877, 28.95967914510724, 28.349574180493544, 27.46789630205484, 26.353179063838752, 25.054140958162098, 23.62755618371937, 22.13577334066696, 20.643990497614553, 19.217405723171822, 17.918367617495164, 16.803650379279073, 15.921972500840363, 15.311867536226664, 15.000000000000028, 15.000000000000032]
#
# radius, xc, yc = least_squares(x_actual, y_actual)
# print radius, xc, yc

# actual center: -0.75, 22.1357733407
# actual radius: 7.17507917513


# calcualte average statistic

a1x = 0.
a1y = 0.
a2x = 0.
a2y = 0.
a3x = 0.
a3y = 0.
a4x = 0.
a4y = 0.

for i in test_measurements[0]:
    a1x += i[0]
    a1y += i[1]

for i in test_measurements[1]:
    a2x += i[0]
    a2y += i[1]

for i in test_measurements[2]:
    a3x += i[0]
    a3y += i[1]

for i in test_measurements[3]:
    a4x += i[0]
    a4y += i[1]

a1x = a1x / len(test_measurements)
a1y = a1y / len(test_measurements)

a2x = a2x / len(test_measurements)
a2y = a2y / len(test_measurements)

a3x = a3x / len(test_measurements)
a3y = a3y / len(test_measurements)

a4x = a4x / len(test_measurements)
a4y = a4y / len(test_measurements)


print "distance between real and prediction 1:", distance_between((target_x, target_y), (a1x, a1y))
print "distance between real and prediction 2:", distance_between((target_x, target_y), (a2x, a2y))
print "distance between real and prediction 3:", distance_between((target_x, target_y), (a3x, a3y))
print "distance between real and prediction 4:", distance_between((target_x, target_y), (a4x, a4y))



turtle.getscreen()._root.mainloop()





