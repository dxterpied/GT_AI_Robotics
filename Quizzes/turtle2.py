from math import *
import random
import turtle
import time

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def getEuclidianDistance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt( (x2 - x1)**2 + (y2 - y1)**2 )


class robot:

    def __init__(self, x = 0.0, y = 0.0, headingAngle = 0.0, turnAngle = pi/5, distance = 1.0):
        self.x = x
        self.y = y
        self.headingAngle = headingAngle
        self.turnAngle = turnAngle # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0


    def move(self, turnAngle, distance, tolerance = 0.001, max_turning_angle = pi):
        # truncate to fit physical limitations
        turningAngle = max( - max_turning_angle, turnAngle)
        turningAngle = min( max_turning_angle, turnAngle)
        distance = max(0.0, distance)
        # Execute motion
        self.headingAngle = self.headingAngle + turnAngle
        #self.headingAngle = angle_trunc(self.headingAngle)
        #self.x += distance * cos(self.headingAngle)
        #self.y += distance * sin(self.headingAngle)
        self.x += cos(self.headingAngle)
        self.y += sin(self.headingAngle)

        return (self.x, self.y)

    def move_in_circle(self):
        return self.move(self.turnAngle, self.distance)

    def sense(self):
        return (random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise))

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)


target_bot = robot(x=0.0, y=0.0, headingAngle = 0.0, turnAngle = pi/2, distance = 1.0)

ctr = 0
size_multiplier= 25.0  #change Size of animation
broken_robot = turtle.Turtle()
broken_robot.resizemode('user')
broken_robot.shapesize(0.5, 0.5, 0.5)
broken_robot.penup()

while ctr <= 4:
    measurement = target_bot.sense()
    x, y = target_bot.move_in_circle()
    #print x, y
    broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
    broken_robot.stamp()

    time.sleep(0.5)
    ctr += 1



