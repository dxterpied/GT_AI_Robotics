
import matplotlib
matplotlib.use('TkAgg')

from filterpy.kalman.sigma_points import *
import turtle
from robot import *
from scipy.linalg import inv, cholesky
from filterpy.kalman import unscented_transform
from filterpy.common import dot3
from numpy import *
from filterpy.kalman import UnscentedKalmanFilter as UKF
import random

# attempts to solve RR5 using UKF from KalmanPy
# based on RR4_UKF.py ---------------------------------------
# trying to see if it works with actual starting x, y, heading and actual turning and distance
# but it still fails...

turtle.setup(800, 800)
window = turtle.Screen()
window.bgcolor('white')
target_robot = turtle.Turtle()
target_robot.shape('turtle')
target_robot.color('green')
target_robot.shapesize(0.2, 0.2, 0.2)
predicted_robot = turtle.Turtle()
predicted_robot.shape('circle')
predicted_robot.color('blue')
predicted_robot.shapesize(0.2, 0.2, 0.2)
measured_robot = turtle.Turtle()
measured_robot.shape('circle')
measured_robot.color('red')
measured_robot.shapesize(0.2, 0.2, 0.2)

target = robot(0.0, 15.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2.0 * target.distance
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -5.0, 0.0)

sigma_vel=0.1
sigma_steer= np.radians(1)
sigma_range= 0.3
sigma_bearing= 0.1

def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x

# state transition function
def fx(x, dt, distance, turning):
    heading = x[2] + turning
    x1 = x[0] + distance * cos(heading)
    y1 = x[1] + distance * sin(heading)
    state = [x1, y1, heading]
    return state


def Hx(sigmas):
    return sigmas[0], sigmas[1]


def residual_h(a, b):
    y = a - b
    for i in range(0, len(y), 2):
        y[i + 1] = normalize_angle(y[i + 1])
    return y


def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y


# sigmas here has three columns - for x, y, and heading
def state_mean(sigmas, Wm):
    x = np.zeros(3)
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = atan2(sum_sin, sum_cos)
    return x


def z_mean(sigmas, Wm):
    z_count = sigmas.shape[1]
    x = np.zeros(z_count)

    for z in range(0, z_count, 2):
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, z+1]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, z+1]), Wm))

        x[z] = np.sum(np.dot(sigmas[:,z], Wm))
        x[z+1] = atan2(sum_sin, sum_cos)
    return x


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

    predictedPosition = [0, 0]
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        turnAngle = []
        ukf = None
    else:
        distances, angles, coords, xy_estimate, steps, turnAngle, ukf= OTHER

        if ukf is None:
            points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
            ukf = UKF(dim_x = 3, dim_z = 2, fx=fx, hx=Hx,
                      dt=1.0, points=points, x_mean_fn=state_mean,
                      z_mean_fn=z_mean, residual_x=residual_x,
                      residual_z=residual_h)

            ukf.x = np.array([target_measurement[0], target_measurement[1], 0.0])
            ukf.P = np.diag([.9, .9, .9])
            # ukf.R = np.diag( [sigma_range**2, sigma_bearing**2] )
            ukf.R = np.diag( [5., 5.] )
            ukf.Q = np.eye(3) * 0.001  # Q must not be zeroes!!! .001 is the best for this case

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


            ukf.predict(dt = 1., fx_args = (1.5, 2*pi / 30))
            z = target_measurement[0], target_measurement[1]
            ukf.update(z)
            xy_estimate = ukf.x[0], ukf.x[1]
            #print "after update", ukf.x[0], ukf.x[1], ukf.x[2]

            newR = robot(ukf.x[0], ukf.x[1], ukf.x[2], rotationSign * avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y
            xy_estimate = newR.x, newR.y

            steps = 1
            while True:
                # check how many steps it will take to get there for Hunter
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1
                newR.move_in_circle()
                xy_estimate = newR.x, newR.y


    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps, turnAngle, ukf)
    if xy_estimate is None:
        xy_estimate = target_measurement

    distance2 = distance_between(hunter_position, predictedPosition)

    if distance2 <= max_distance:
        turning = angle_trunc(get_heading(hunter_position, predictedPosition) - hunter_heading)
        distance = distance2
        OTHER = (distances, angles, coords, None, steps, turnAngle, ukf)
    else:
        turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading)
        distance = distance_between(hunter_position, xy_estimate)


    return turning, distance, OTHER



# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)



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

    x = []
    y = []
    x_actual = []
    y_actual = []

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
    prediction.shape('circle')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    #prediction.penup()
    #broken_robot.penup()
    #End of Visualization
    broken_handle = 0.
    prediction_handle = 0.

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
        x.append(target_measurement[0])
        y.append(target_measurement[1])
        x_actual.append(target_bot.x)
        y_actual.append(target_bot.y)

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

        broken_robot.clearstamp(broken_handle)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_handle = broken_robot.stamp()

        prediction.clearstamp(prediction_handle)
        prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
        prediction_handle = prediction.stamp()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."

    # print x
    # print y
    # print x_actual
    # print y_actual
    return caught


#demo_grading(hunter, target, next_move_straight_line)
demo_grading_visual(hunter, target, next_move_straight_line)


# scores = []
# fails = 0
# for i in range(1000):
#     print i
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




#turtle.getscreen()._root.mainloop()