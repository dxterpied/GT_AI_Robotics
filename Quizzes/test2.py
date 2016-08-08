# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

sigma = 2.0

def predicate_mean(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    clockwise = None
    if not OTHER:
        OTHER = []
    OTHER.append(measurement)
    if len(OTHER) == 1:
        x = OTHER[0][0]
        y = OTHER[0][1]
        xy_estimate = (x, y)
    elif len(OTHER) == 2:
        x1 = OTHER[0][0]
        y1 = OTHER[0][1]
        x2 = OTHER[1][0]
        y2 = OTHER[1][1]
        dx = x2 - x1
        dy = y2 - y1
        xy_estimate = (dx+x2, dy+y2)
    else:
        headings = []
        dists = []
        edges = []
        for i in xrange(1, len(OTHER)):
            p1 = (OTHER[i][0], OTHER[i][1])
            p2 = (OTHER[i-1][0], OTHER[i-1][1])
            dist = distance_between(p1, p2)
            dx = p1[0] - p2[0]
            dy = p1[1] - p2[1]
            edges.append(dy*dx)
            heading = atan2(dy, dx)
            dists.append(dist)
            headings.append(heading)

        # find turning wise
        clockwise = True
        if sum(edges) < 0:
            clockwise = False

        turnings = []
        for i in xrange(1, len(headings)):
            turning = headings[i] - headings[i-1]
            if clockwise:
                if turning > 0:
                    turning -= 2 * pi
            else:
                if turning < 0:
                    turning += 2 * pi
            turnings.append(turning)

        est_dist = sum(dists) / len(dists)
        est_turning = sum(turnings) / len(turnings)
        est_heading = angle_trunc(headings[-1] + est_turning)
        x = OTHER[-1][0]
        y = OTHER[-1][1]
        est_x = x + est_dist * cos(est_heading)
        est_y = y + est_dist * sin(est_heading)
        xy_estimate = (est_x, est_y)

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    return xy_estimate, OTHER


# u = matrix([[0.], [0.], [0.]])  # external motion
F = matrix([[1., 1., 0.],
            [0., 1., 0.],
            [0., 0., 1.]])      # next state function
H = matrix([[1., 0., 0.],
            [0., 0., 1.]])      # measurement function
R = matrix([[sigma , 0.],
            [0., sigma]])          # measurement uncertainty
I = matrix([[1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 1.]])      # identity matrix


def predicate_kalman(measurement, OTHER=None):
    if not OTHER:
        x = matrix([[0.],
                    [0.],
                    [0.]])  # initial state (location and velocity)
        P = matrix([[1000., 0., 0.],
                    [0., 1000., 0.],
                    [0., 0., 1000.]])  # initial uncertainty
        OTHER = { 'm': [], 'x': x, 'P': P }
    OTHER['m'].append(measurement)

    # calculate heading and distance from previous data
    if len(OTHER['m']) == 1:
        m_heading = 0
        m_distance = 0
    else:
        p1 = (OTHER['m'][-1][0], OTHER['m'][-1][1])
        p2 = (OTHER['m'][-2][0], OTHER['m'][-2][1])
        m_distance = distance_between(p1, p2)
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        m_heading = atan2(dy, dx) % (2 * pi)

    # Implement kalman filter
    x = OTHER['x']
    P = OTHER['P']
    pre_heading = x.value[0][0]
    for d in [-1, 0, 1]:
        diff = (int(pre_heading / (2 * pi)) + d) * (2 * pi)
        if abs(m_heading + diff - pre_heading) < pi:
            m_heading += diff
            break
    # measurement update
    y = matrix([[m_heading],
                [m_distance]]) - H * x
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - K * H) * P
    # prediction
    x = F * x
    P = F * P * F.transpose()
    OTHER['x'] = x
    OTHER['P'] = P

    # Predicate current position
    if x.value[1][0] == 0.0:
        current_x = measurement[0]
        current_y = measurement[1]
    else:
        n_pie = int((2 * pi) / abs(x.value[1][0]))
        m_len = len(OTHER['m'])
        p_index = (m_len - 1) % n_pie
        points = []
        while p_index < m_len:
            points.append(OTHER['m'][p_index])
            p_index += n_pie
        points_x, points_y = zip(*points)
        current_x = sum(points_x) / len(points_x)
        current_y = sum(points_y) / len(points_y)

    est_heading = x.value[0][0]
    est_distance = x.value[2][0]
    est_next = next_position_in_circle(current_x, current_y,
                                       est_heading, est_distance)
    return est_next, OTHER


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.
    est_target_pos, OTHER = predicate_kalman(target_measurement, OTHER)
    est_distance = distance_between(hunter_position, est_target_pos)
    steps = 1
    x = OTHER['x']
    while est_distance > steps * max_distance:
        x = F * x
        est_target_pos = next_position_in_circle(est_target_pos[0], est_target_pos[1],
                                                 x.value[0][0], x.value[2][0])
        est_distance = distance_between(hunter_position, est_target_pos)
        steps += 1

    heading_to_target = get_heading(hunter_position, est_target_pos)

    turning = heading_to_target - hunter_heading
    turning = angle_trunc(turning)

    if est_distance > max_distance:
        distance = max_distance  # full speed ahead!
    else:
        distance = est_distance
    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    return turning, distance, OTHER


def next_position_in_circle(x, y, heading, distance):
    est_x = x + distance * cos(heading)
    est_y = y + distance * sin(heading)
    return est_x, est_y

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance  # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance  # hunter must be within 0.02 step size to catch target
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
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance,
                                                 OTHER)

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

def demo_grading_visualized(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change Size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
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
        #Visualize it
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


def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER  # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = sigma * target.distance # VERY NOISY!! # Part 5
# measurement_noise = .05*target.distance # Part 4
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)


demo_grading_visualized(hunter, target, next_move)
#demo_grading(hunter, target, next_move)