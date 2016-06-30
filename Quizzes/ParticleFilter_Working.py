# Please only modify the indicated area below!

from math import *
import random
import turtle    #You need to run this locally to use the turtle module

window = turtle.Screen()
window.screensize(800, 800)
window.bgcolor('white')
target_robot = turtle.Turtle()
target_robot.shape('turtle')
target_robot.color('green')
target_robot.shapesize(0.1, 0.1, 0.1)
target_robot.penup()
hunter_robot = turtle.Turtle()
hunter_robot.shape('circle')
hunter_robot.color('blue')
hunter_robot.shapesize(0.1, 0.1, 0.1)
hunter_robot.penup()

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
world_size = 100.0
size_multiplier= 10.0  #change Size of animation


class robot:
    def __init__(self, x=0.0, y=0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.distance_noise = 0.0;
        self.turning_noise    = 0.0;
        self.measurement_noise   = 0.0;

    def set_noise(self, distance_noise, turning_noise, measurement_noise):
        self.distance_noise = float(distance_noise);
        self.turning_noise    = float(turning_noise);
        self.measurement_noise   = float(measurement_noise);


    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.measurement_noise)
            Z.append(dist)
        return Z


    def move(self, turning, distance):
        heading = (self.heading + turning + random.gauss(0.0, self.turning_noise)) % (2*pi)
        dist = distance + random.gauss(0.0, self.distance_noise)
        x = self.x + (cos(heading) * dist)
        y = self.y + (sin(heading) * dist)
        # create new particle
        newRobot = robot(x, y, heading, turning, distance)
        newRobot.set_noise(self.distance_noise, self.turning_noise, self.measurement_noise)
        return newRobot

    def move_in_circle(self):
        return self.move(self.turning, self.distance)

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


    def measurement_prob(self, measurement):
        # calculates how likely a measurement should be
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.measurement_noise, measurement[i])
        return prob


    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.heading))


def get_position(p):
    x = y = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
    return [x / len(p), y / len(p)]


def distance_between(point1, point2):
    """Computes Euclidean distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)



turning = 0.1
distance = 2.0
distance_tolerance = 0.01 * distance

myrobot  = robot(x=0.0, y=0.0, heading = 0.5, turning = 2*pi/34.0, distance =  1.5)
# if turning in particles is different from turning in target, it all goes kaboom.....
#myrobot = robot(x=0.0, y=0.0, heading = 0.5, turning = turning, distance = distance)
measurement_noise = 0.05 * myrobot.distance
myrobot.set_noise(distance_noise=0.0, turning_noise=0.0, measurement_noise=measurement_noise)

N = 1000
T = 1000

# create new particles
p = []
for i in range(N):
    r = robot(random.random() * world_size,
              random.random() * world_size,
              random.random() * 2.0*pi,
              turning = turning,
              distance = distance) # use random initialization
    r.set_noise(distance_noise=0.05, turning_noise=0.05, measurement_noise=2.0)
    p.append(r)


ctr = 1
for t in range(T):

    myrobot = myrobot.move_in_circle()
    Z = myrobot.sense()

    target_robot.goto(myrobot.x * size_multiplier, myrobot.y * size_multiplier - 200)
    target_robot.stamp()

    p2 = []
    for i in range(N):
        p2.append(p[i].move_in_circle())

    p = p2

    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))

    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3

    predicted_position = get_position(p)
    error = distance_between( (predicted_position[0], predicted_position[1]), (myrobot.x, myrobot.y))
    hunter_robot.goto(predicted_position[0] * size_multiplier, predicted_position[1] * size_multiplier - 200)
    hunter_robot.stamp()

    if error <= distance_tolerance:
        print "You got it right! It took you ", ctr, " steps to localize."
        break
    ctr += 1


turtle.getscreen()._root.mainloop()



