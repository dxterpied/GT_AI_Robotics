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
visual = False
gains = [   0.09000000000000001,    # p distance
            0.5100000000000009,     # p heading
            0.0010499999999999993,  #i distance
            0.0005,                 #i heading
            8.5275554301e-05,       #d distance
            -0.009999999999999995]  #d heading


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves.

    if OTHER == None:
        OTHER = {}
        OTHER["hunter_positions"] = [hunter_position]
        OTHER["hunter_headings"] = [hunter_heading]
        OTHER["target_measurements"] = [target_measurement]
        OTHER["distance_integral"] =  0
        OTHER["heading_integral"] = 0
        OTHER["previous_point"] = (0.0,0.0)
        OTHER["previous_heading"] = 0
        OTHER["distance_between_array"] = []
    else:
        OTHER["hunter_positions"].append(hunter_position)
        OTHER["hunter_headings"].append(hunter_heading)
        OTHER["target_measurements"].append(target_measurement)

    predicted_position, OTHER = estimate_future_pos(target_measurement,7,OTHER)
    next_position = predicted_position
    dx = next_position[0] - hunter_position[0]
    dy = next_position[1] - hunter_position[1]
    #print dx,dy
    aR = atan2(dy,dx)
    heading_error = angle_trunc(aR - hunter_heading)
    distance_error = distance_between(hunter_position,next_position)

    if (distance_error < .001 * max_distance):
        distance_guess = OTHER["distance_integral"]
        heading_guess = OTHER["heading_integral"]
    else:
        distance_guess = distance_error + OTHER["distance_integral"]
        heading_guess = heading_error + OTHER["heading_integral"]

    OTHER["distance_integral"]= distance_guess
    OTHER["heading_integral"]= heading_guess
    distance_difference = OTHER["distance_integral"] - distance_error
    heading_difference = OTHER["heading_integral"] - heading_guess

    distance = gains[0] * distance_error + gains[2] * distance_guess - gains[4] * distance_difference
    turning =  gains[1] * heading_error + gains[3] * heading_guess + gains[5] * heading_difference
    return turning, distance, OTHER


def mean(lists):
    sums = 0
    for i in range(len(lists)):
        sums += lists[i]
    return sums/len(lists)


def estimate_future_pos(measurement,num_predicted_steps, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    """
    OTHER VECTOR FORM:
    [
       previousPoint (x,y) # this is the previous measurement,
       prev_heading (int) # this is the heading - the turn angle,
       distance_between_array([])
   ]
    """
    #going to set this section above
    # if OTHER == None:
    #     OTHER = {}
    #     OTHER["previous_point"] = (0.0,0.0)
    #     OTHER["previous_heading"] = 0
    #     OTHER["distance_between_array"] = []

    previous_heading = OTHER["previous_heading"]
    previous_point = OTHER["previous_point"]#retrieving the previous point to use later
    distance_between_array = OTHER["distance_between_array"]

    dx = measurement[0] - previous_point[0]
    dy = measurement[1] - previous_point[1]
    aR = atan2(dy,dx)
    distance_between_array.append(distance_between(previous_point,measurement))
    future_distance = mean(distance_between_array)*num_predicted_steps

    turn_angle = aR - previous_heading
    heading = aR + turn_angle
    est_x = measurement[0] + future_distance * cos(heading)
    est_y = measurement[1] + future_distance * sin(heading)

    OTHER["previous_point"] = measurement #storing the measurement as the previous point
    OTHER["previous_heading"] = aR
    OTHER["distance_between_array"] = distance_between_array
    xy_estimate = (est_x,est_y )
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    return xy_estimate, OTHER

#this calculates the angle between 2 points using the law of cosines
def angle_between(p1, p2):
    x1,y1 = p1[0],p1[1]
    x2,y2 = p2[0],p2[1]
    return acos((x1*x2 + y1*y2) / (sqrt(x1**2 + y1**2) * sqrt(x2**2 + y2**2)))

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


if visual == False:
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
else:
    def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
        """Returns True if your next_move_fcn successfully guides the hunter_bot
        to the target_bot. This function is here to help you understand how we
        will grade your submission."""
        max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
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
        size_multiplier = 15.0 #change size of animation
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
                return 1000
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

print demo_grading(hunter, target, next_move)

scores = []
for i in range(10000):
    hunter = robot(-10.0, -20.0, 0.0)
    scores.append(demo_grading(hunter, target, next_move))
print "average score: ", sum(scores)/len(scores)
print "minimum score: ", min(scores)
print "maximum score: ", max(scores)
