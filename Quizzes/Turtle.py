from math import *
import random
import turtle    #You need to run this locally to use the turtle module
import time

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:

    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        """This function is called when you create a new robot. It sets some of
        the attributes of the robot, either to their default values or to the values
        specified when it is created."""
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0


    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
        helpful when using particle filters."""
        self.turning_noise    = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)


    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):
        """This function turns the robot and then moves it forward."""
        # apply noise, this doesn't change anything if turning_noise
        # and distance_noise are zero.
        turning = random.gauss(turning, self.turning_noise)
        distance = random.gauss(distance, self.distance_noise)
        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)

        # Execute motion
        self.heading += turning
        self.heading = angle_trunc(self.heading)
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)

        return (self.x, self.y)

    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        return self.move(self.turning, self.distance)

    def sense(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to,
        but not necessarily equal to, the robot's (x, y) position."""
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading_new(target_bot, OTHER = None):
    ctr = 0
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation

    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('blue')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.5, 0.5, 0.5)

    #broken_robot.penup()

    while ctr <= 10:
        ctr += 1
        measurement = target_bot.sense()
        x, y = target_bot.move_in_circle()
        #print x, y
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_robot.stamp()

        time.sleep(0.5)

# This is how we create a target bot. Check the robot.py file to understand
test_target = robot(x=5.0, y=5.0, heading = 0.0, turning = 2*pi/10, distance = 1.0)

demo_grading_new(test_target)




