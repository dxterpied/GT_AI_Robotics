import matplotlib
matplotlib.use('TkAgg')

from filterpy.kalman.sigma_points import *
import turtle
from robot import *
from scipy.linalg import inv, cholesky
from filterpy.kalman import unscented_transform
from filterpy.common import dot3
from numpy import *

# Ilya:
# this one uses circular regression.
# Circular regression is not complete yet. I am trying to think of some other ideas.

size_multiplier = 25.
target = robot(0.0, 0.0, 0.0, 2*pi / 30, 1.5)
target.set_noise(0.0, 0.0, 2.0 * target.distance)
hunter = robot(-10.0, -20.0, 0.0)

bumblebee = turtle.Turtle()
bumblebee.shape('square')
bumblebee.color('black')
bumblebee.penup()
bumblebee.shapesize(0.2, 0.2, 0.2)

hunterbee = turtle.Turtle()
hunterbee.shape('turtle')
hunterbee.color('brown')
hunterbee.shapesize(0.3, 0.3, 0.3)


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


def next_move_straight_line(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    xy_estimate = target_measurement

    if OTHER is None:
        distances = []
        distances.append(1.5) # just append an estimate for now
        angles = []
        coords = []
        xy_estimate = target_measurement
        turnAngle = []
        x = []
        x.append(target_measurement[0])
        y = []
        y.append(target_measurement[1])
        estimated_coords = []
    else:
        distances, angles, coords, xy_estimate, turnAngle, x, y, estimated_coords = OTHER

        # collect measurements
        x.append(target_measurement[0])
        y.append(target_measurement[1])

        if len(coords) >= 2:

            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = target_measurement
            turnAngle.append(calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1]))
            rotationSign = getRotationSign(turnAngle)

            # y1Delta = point2[1] - point1[1]
            # hypotenuse1 = distance_between(point1, point2)
            # headingAngleAvg1 = asin(y1Delta / hypotenuse1)
            #
            # y2Delta = point3[1] - point2[1]
            # x2Delta = point3[0] - point2[0]
            # hypotenuse2 = distance_between(point2, point3)
            # headingAngle2 = atan2(y2Delta, x2Delta)
            # headingAngleAvg2 = asin(y2Delta / hypotenuse2)
            # predictedTurnAngleAvg = headingAngleAvg2 - headingAngleAvg1

            #angles.append(abs(predictedTurnAngleAvg))
            #avgAngle = sum(angles)/len(angles)

            # start only after n measurements
            if len(coords) > 20:
                # find radius and center coords
                radius, xc, yc = least_squares(x, y)

                # get the heading to measurement based on predicted center
                xcDelta = target_measurement[0] - xc
                ycDelta = target_measurement[1] - yc
                angle = atan2(ycDelta, xcDelta)

                # get new estimated location on the circumference based on the above angle
                estimated_x = xc + radius * cos(angle)
                estimated_y = yc + radius * sin(angle)

                # get previous coords
                if len(estimated_coords) > 0:
                    prev_x, prev_y = estimated_coords[len(estimated_coords) - 1]
                    estimated_coords.append((estimated_x, estimated_y))

                    # heading for previous estimated location
                    xcDelta2 = prev_x - xc
                    ycDelta2 = prev_y - yc
                    angle2 = atan2(ycDelta2, xcDelta2)
                    predictedTurnAngle = angle_trunc(angle2 - angle)

                    #predict next position
                    # estimated_x = xc + radius * cos(predictedTurnAngle)
                    # estimated_y = yc + radius * sin(predictedTurnAngle)

                    # get distance between previous estimated and current estimated location
                    distance = distance_between((prev_x, prev_y), (estimated_x, estimated_y))
                    #distance = distance * angle / abs(angle) # get the distance sign correctly (negative or positive) based on angle
                    distances.append(distance)
                    avgDT = sum(distances)/len(distances)

                    print "avgDT", avgDT

                    newR = robot(estimated_x, estimated_y, angle, rotationSign * predictedTurnAngle, avgDT)
                    newR.move_in_circle()
                    predictedPosition = newR.x, newR.y
                    xy_estimate = newR.x, newR.y

                # if len(coords) >= 44:
                    #print "angle", angle, "angle2", angle2 # the angles are the same
                    # bumblebee.goto(xc * size_multiplier, yc * size_multiplier - 200)
                    # bumblebee.goto(estimated_x * size_multiplier, estimated_y * size_multiplier - 200)
                    # bumblebee.stamp()

                # try to find the shortest straight path from hunter position to predicted target position
                # steps = 1
                # while True:
                #     # check how many steps it will take to get there for Hunter
                #     if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                #         break
                #     steps += 1
                #     newR.move_in_circle()
                #     xy_estimate = newR.x, newR.y
                else:
                    estimated_coords.append((estimated_x, estimated_y))


    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, turnAngle, x, y, estimated_coords)
    turning = angle_trunc( get_heading(hunter_position, xy_estimate) - hunter_heading ) # turn towards the target
    distance = distance_between(hunter_position, xy_estimate)

    # if len(coords) >= 45:
    #     print "turning", turning, "distance", distance
    #     hunterbee.goto(hunter_position[0] * size_multiplier, hunter_position[1] * size_multiplier - 200)
    #     hunterbee.stamp()
    #     bumblebee.goto(xy_estimate[0] * size_multiplier, xy_estimate[1] * size_multiplier - 200)
    #     bumblebee.stamp()


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

# x_points = []
# y_points = []
# x_actual = []
# y_actual = []

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
    broken_robot.penup()

    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)


    noise = turtle.Turtle()
    noise.shape('circle')
    noise.color('red')
    noise.resizemode('user')
    noise.shapesize(0.2, 0.2, 0.2)
    #noise.penup()

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

        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_robot.stamp()

        prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
        prediction.stamp()

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
#


turtle.getscreen()._root.mainloop()





