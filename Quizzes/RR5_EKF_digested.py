
# taken from https://github.com/lhaberke/AI_Robotics_FinalProject/StudentMainBonus.py

# ----------
# Part Five
#

from robot import *
from math import *
import turtle
import matplotlib
matplotlib.use('TkAgg')
from numpy import eye, diag, sin, cos, linalg, pi, matrix



# Ilya: this one works sometimes - around 23% success. The closest of all for RR5 so far.

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    noise_est = 60. # should be greater than noise variance
    if not OTHER: # first time calling this function, set up my OTHER variables.
        last_est_xy = None #target_measurement[:]
        X = None
        P = None
        OTHER = [X, P]
    else: # not the first time, update my history
        X, P = OTHER

    est_target_xy, X, P = EKF_UPDATE(target_measurement, X, P, 1., noise_est)

    # Best guess as to true target coordinates now
    next_est_target_xy, X, P = EKF_PREDICT(X, P, dt=1.)

    # Uses new estimate to predict the next estimated target location
    hunter_to_xy = next_est_target_xy # works if target will be within range
    dist_to_target = distance_between(next_est_target_xy, hunter_position)
    X_next, P_next = X.copy(), P.copy()

    for D in range(int(dist_to_target / (max_distance))):
        # to catch target, look ahead D moves and go that way
        # Don't update P since we have no real information to update with
        hunter_to_xy, X_next, _ = EKF_PREDICT(X_next, P_next, 1.)

    turning = angle_trunc(get_heading(hunter_position, hunter_to_xy) - hunter_heading)
    distance = min(dist_to_target, max_distance)
    OTHER = [X, P]
    return turning, distance, OTHER


def EKF_PREDICT(X = None, P = None, dt = 0.):

    if not dt: dt = 1.0 # time step

    max_speed = 1.5 # taken from problem in this case
    max_turn_rate = pi/8 # max of 22.5deg/sec

    # Various motion noise for Q
    x_var = y_var = max_speed*dt    # set for max speed
    heading_var = max_turn_rate*dt    # Assuming max turn in a step
    v_var = max_speed               # set for max speed
    turning_var = .05               # assuming low acceleration
    # Q is the Motion Uncertainty Matrix, I'll use max step changes for now.
    #       Assuming no correlation to motion noise for now
    Q = diag([x_var**2, y_var**2, heading_var**2, v_var**2, turning_var**2])

    if type(X) == type(None): # Initialize X statespace
        X = matrix([[0.],  # x
                    [0.],  # y
                    [0.],  # heading
                    [0.],  # velocity
                    [0.]]) # turning (positive is counter clockwise)
    if type(P) == type(None): # Initialize Uncertainty Matrix - no correlation uncertainty
        P = diag([1000., 1000., 2*pi, 100., 2*pi])

    # Break out statespace for readability
    x, y, heading, velocity, turning = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0]

    if abs(turning) < 0.0001: # Avoid divide by zero, use as if no turning
        # Using a linear FX for this case of no turning
        FX = matrix([[x + velocity * dt * cos(heading)],   # basic no turn geometry
                     [y + velocity * dt * sin(heading)],   # basic no turn geometry
                     [        heading          ],   # no turning so heading = heading
                     [        velocity         ],   # velocity is constant
                     [     0.0000001         ]])  # Avoid divide by zero in JF
    else: # Take turning into account with nonlinear FX
        # FX is the nonlinear F(X) that predicts motion update
            # x = x + integral(v*cos(heading + turning*dt) - v*cos(heading))
            # y = y + integral(v*sin(heading + turning*dt) - v*sin(heading))
            # heading = heading + turning*dt
        FX = matrix([[x + velocity/turning * ( sin(heading + turning*dt) - sin(heading))],
                    [ y + velocity/turning * (-cos(heading + turning*dt) + cos(heading))],
                    [                   heading + turning*dt                   ],
                    [                           velocity                       ],
                    [                       turning                         ]])

    # Since X = F(X), we can just set X = FX
    X = FX

    # Break out new estimated statespace for readability
    x, y, heading, velocity, turning = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0]

    # JF is the linearized F(X) matrix, to get JF(X), we do partial derivatives
    #        |  dF(X[0])/dX[0]  ...  dF(X[n])/dX[0]  |
    #   JF = |       ...                   ...       |
    #        |  dF(X[0])/dX[n]  ...  dF(X[n])/dX[n]  |
    # Notice diagonals will all be 1 and lower triangle has no correlation (=0)
    JF = eye(5)
    JF[0,2] = velocity/turning * (cos(heading + turning*dt) - cos(heading))
    JF[0,3] = 1./turning * (sin(heading + turning*dt) - sin(heading))
    JF[0,4] = - velocity/(turning**2) * (sin(heading + turning*dt) - sin(heading)) + velocity/turning * dt * cos(heading + turning*dt)
    JF[1,2] = velocity/turning * (sin(heading + turning*dt) - sin(heading))
    JF[1,3] = 1./turning * (-cos(heading + turning*dt) + cos(heading))
    JF[1,4] = - velocity/(turning**2) * (-cos(heading + turning*dt) + cos(heading)) + velocity/turning * dt * sin(heading + turning*dt)
    JF[2,4] = dt

    # Update Probability Matrix
    P = JF * P * JF.T + Q

    estimate_xy = [X[0,0], X[1,0]]

    return estimate_xy, X, P


def EKF_UPDATE(measurement=[0.,0.], X=None, P=None, dt=0, noise_est=0):
    # Extended Kalman Filter Measurement Estimate for nonlinear X state
    #       I am modeling with a constant velocity and yaw rate

    # How much measurement noise to account for?
    #   Higher means rely more on average of data and motion prediction
    #   Try to set low but high enough for estimates not to diverge.
    #   ie try 2x-5x the gauss variation
    if noise_est: xy_noise_var = noise_est
    else: xy_noise_var = 20.

    if not dt: dt = 1.0 # time step

    if type(X) == type(None): # Initialize X statespace
        X = matrix([[0.],  # x
                    [0.],  # y
                    [0.],  # heading (x_dir is 0 deg, y_dir is 90 deg)
                    [0.],  # velocity
                    [0.]]) # turning (positive is counter clockwise)
    if type(P) == type(None): # Initialize Uncertainty Matrix - no correlation uncertainty
        P = diag([1000., 1000., 2.*pi, 100., 2.*pi])

    # Z is the measurement itself, currently only measure x,y
    Z = matrix([[float(measurement[0])],
                [float(measurement[1])],
                [         0.          ],
                [         0.          ],
                [         0.          ]]) # for heading, etc

    # Break out statespace for readability
    x, y, heading, velocity, turning = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0]

    # HF is the nonlinear (linear if only gps sensor) measurement matrix
    HF = matrix([[x],  # Only measures x, y.
                 [y],
                 [0.],
                 [0.],
                 [0.]])

    # JH is linearized jacobian of H
    JH = matrix([[1., 0., 0., 0., 0.],  # x row
                 [0., 1., 0., 0., 0.],
                 [0., 0., 0., 0., 0.],
                 [0., 0., 0., 0., 0.],
                 [0., 0., 0., 0., 0.]]) # y row. add rows for more sensors

    # R is the measurement noise matrix.  Using problem's x,y noise.
    R = matrix([[xy_noise_var**2, 0., 0., 0., 0.],
                [0., xy_noise_var**2, 0., 0., 0.],
                [0.,    0., 0.001,    0.,     0.],
                [0.,    0.,    0., 0.001,     0.],
                [0.,    0.,    0.,    0.,  0.001]])

    I = eye(5)
    S = JH * P * JH.T + R
    # Kalman factor
    K = (P * JH.T) * linalg.inv(S)
    # Y is the error matrix (measurement - estimate)
    Y = Z - HF
    X = X + (K * Y)
    # Probability matrix will get more precise with measurement
    P = (I - (K * JH)) * P

    estimate_xy = [X[0,0], X[1,0]]

    return estimate_xy, X, P


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(from_position, to_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    from_x, from_y = from_position
    to_x, to_y = to_position
    heading = atan2(to_y - from_y, to_x - from_x)
    heading = angle_trunc(heading)
    return heading


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def turtle_demo(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    size_multiplier = 20.0 #change size of animation

    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)


    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('black')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    broken_robot.pendown()

    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()


    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)

    EKF_broken_robot = turtle.Turtle()
    EKF_broken_robot.shape('turtle')
    EKF_broken_robot.color('red')
    EKF_broken_robot.penup()
    EKF_broken_robot.resizemode('user')
    EKF_broken_robot.shapesize(0.1, 0.1, 0.1)


    chaser_robot.pendown()
    EKF_broken_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    min_sep = 9999
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        min_sep = min(min_sep,separation)

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

        #measured_estimate = OTHER[0]
        # EKF_broken_robot.setheading(target_bot.heading*180/pi)
        # EKF_broken_robot.goto(measured_estimate[0]*size_multiplier, measured_estimate[1]*size_multiplier-100)
        # EKF_broken_robot.stamp()
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
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
        #print 'separation: %f, turning: %f, distance: %f, %d' % (separation, turning, distance, .555)
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


target = robot(0.0, 0.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2.0*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -20.0, 0.0)

#print demo_grading(hunter, target, next_move)
turtle_demo(hunter, target, next_move)#, None)

# scores = []
# fails = 0
# for i in range(1000):
#     print i
#     target = robot(0.0, 0.0, 0.0, 2*pi / 30, 1.5)
#     target.set_noise(0.0, 0.0, measurement_noise)
#     hunter = robot(-10.0, -20.0, 0.0)
#     score = demo_grading(hunter, target, next_move)
#     if score == 1000:
#         fails += 1
#     else:
#         scores.append(score)
#
# print "average score: ", sum(scores)/ float(len(scores))
# print "minimum score: ", min(scores)
# print "maximum score: ", max(scores)
# print "fails: ", fails

# for 100 runs:
# average score:  447.869565217
# minimum score:  46
# maximum score:  989
# fails:  77







