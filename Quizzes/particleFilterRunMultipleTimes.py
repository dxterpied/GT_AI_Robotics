# In this exercise, write a program that will
# run your previous code twice.
# Please only modify the indicated area below!

from math import *
import random
import turtle
import time

landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0

class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)


    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);


    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z


    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'

        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size

        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, x):

        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


    def measurement_prob(self, measurement):

        # calculates how likely a measurement should be

        prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


# returns the arithmetic means of x, y and orientation. It is already weighted.
def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi))
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]


####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER/MODIFY CODE BELOW ####
myrobot = robot()
myrobot.set(50, 50, myrobot.orientation)

# window = turtle.Screen()
# window.bgcolor('white')
# size_multiplier= 4.0  #change Size of animation
#
# target = turtle.Turtle()
# target.color('blue')
# target.shape('circle')
# target.shapesize(0.3, 0.3, 0.3)


N = 10000
T = 7
p = []
turtles = []

for i in range(N):
    x = robot()
    x.set_noise(0.05, 0.05, 5.0)
    p.append(x)
    # t = turtle.Turtle()
    # t.color('red')
    # t.shapesize(0.2, 0.2, 0.2)
    # t.penup()
    # turtles.append(t)


for t in range(T):
    # if t == 9:
        # window.clear()

    # move the target
    myrobot = myrobot.move(0.1, 5.0)
    Z = myrobot.sense()

    # target.goto(myrobot.x * size_multiplier, myrobot.y * size_multiplier)
    # target.stamp()

    p2 = []

    # move all particles
    for i in range(N):
        r = p[i]
        r.move(0.1, 5.0)
        p2.append(r)

        # turtle = turtles[i]
        # if t == 4:
        #     turtle.color('black')
        #     turtle.goto(r.x * size_multiplier, r.y * size_multiplier)
        #     turtle.stamp()
    # if t == 9:
    #     time.sleep(10)
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
    print "round ", t
    # window.clear()

print "prediction: ", get_position(p) #Leave this print statement for grading purposes!
print "actual position: ", myrobot

# turtle.getscreen()._root.mainloop()



