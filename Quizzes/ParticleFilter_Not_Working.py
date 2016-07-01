from robot import *
from math import *
import random
from numpy import *
import time
import turtle

window = turtle.Screen()
window.screensize(800, 800)
window.bgcolor('white')
target_robot = turtle.Turtle()
target_robot.shape('turtle')
target_robot.color('green')
target_robot.shapesize(0.1, 0.1, 0.1)
target_robot.penup()
hunter_robot = turtle.Turtle()
hunter_robot.shape('arrow')
hunter_robot.color('blue')
hunter_robot.shapesize(0.1, 0.1, 0.1)
hunter_robot.penup()
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
world_size = 100.0
size_multiplier= 15.0  #change Size of animation
turning = 2*pi/34.0
distance = 1.5
distance_tolerance = 0.01 * distance
N = 1000
T = 1000
measurement_noise = 5.0


myrobot = robot(x=0.0, y=0.0, heading = 0.5, turning = turning, distance =  distance)
myrobot.set_noise(new_t_noise=0.0, new_d_noise=0.0, new_m_noise=0.05 * myrobot.distance)
particles = []



def measurement_prob(particleX, particleY, targetMeasurement):
    # calculates how likely a measurement should be
    prob = 1.0;
    for i in range(len(landmarks)):
        dist = distance_between( (particleX, particleY),  (landmarks[i][0], landmarks[i][1]) )
        prob *= Gaussian(dist, measurement_noise, targetMeasurement[i])
    return prob


# this sense is only used for target bot
def senseToLandmarks(targetX, targetY, measurement_noise):
    Z = []
    import random
    for i in range(len(landmarks)):
        dist = distance_between( (targetX, targetY),  (landmarks[i][0], landmarks[i][1]) )
        dist += random.gauss(0.0, measurement_noise)
        Z.append(dist)
    return Z


def Gaussian(mu, sigma, x):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


# returns the arithmetic means of x, y and orientation. It is already weighted.
def get_position(p):
    x = y = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
    return [ x/len(p), y/len(p) ]


def move(x, y, turning, distance, heading, distance_noise, turning_noise, measurement_noise):
    import random

    newHeading = (heading + turning + random.gauss(0.0, turning_noise)) % (2*pi)
    dist = distance + random.gauss(0.0, distance_noise)
    newX = x + (cos(newHeading) * dist)
    newY = y + (sin(newHeading) * dist)
    # create new particle
    newRobot = robot(newX, newY, newHeading, turning, distance)

    newRobot.set_noise(new_t_noise = turning_noise,
                new_d_noise = distance_noise,
                new_m_noise = measurement_noise) # measurement noise is not used in particles

    return newRobot


"""Computes distance between point1 and point2. Points are (x, y) pairs."""
def distance_between(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)




def estimate_next_pos(measurement, OTHER = None):
    global particles

    Z = senseToLandmarks(measurement[0], measurement[1], 0.05 * myrobot.distance)

    xy_estimate = particle_filter(Z)

    OTHER = (None, None, None)

    return xy_estimate, OTHER


def particle_filter(targetMeasurementToLandmarks):
    global particles

    # PREDICT by moving
    p2 = []
    for i in range(N):
        newParticle = move(particles[i].x, particles[i].y, particles[i].turning, particles[i].distance, particles[i].heading, particles[i].distance_noise, particles[i].turning_noise, particles[i].measurement_noise)
        p2.append(newParticle)
    particles = p2

    # UPDATE by creating weights
    w = []
    for i in range(N):
        particle = particles[i]
        mp = measurement_prob( particle.x, particle.y, targetMeasurementToLandmarks)
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


# create new particles
for i in range(N):
    r = robot(random.random() * world_size,
              random.random() * world_size,
              random.random() * 2.0*pi, # noise in orientation
              turning = turning,
              distance = distance)
    r.set_noise(new_t_noise = 0.05,
                new_d_noise = 0.05,
                new_m_noise = measurement_noise) # measurement noise is not used in particles

    particles.append(r)



# ctr = 1
# for t in range(T):
#
#     myrobot.move_in_circle()
#     Z = senseToLandmarks(myrobot.x, myrobot.y, myrobot.measurement_noise)
#
#     target_robot.goto(myrobot.x * size_multiplier, myrobot.y * size_multiplier - 200)
#     target_robot.stamp()
#
#     # PREDICT by moving
#     p2 = []
#     for i in range(N):
#         newParticle = move(particles[i].x, particles[i].y, particles[i].turning, particles[i].distance, particles[i].heading, particles[i].distance_noise, particles[i].turning_noise, particles[i].measurement_noise)
#         p2.append(newParticle)
#     particles = p2
#
#     # UPDATE by creating weights
#     w = []
#     for i in range(N):
#         particle = particles[i]
#         mp = measurement_prob( particle.x, particle.y, Z)
#         w.append(  mp )
#
#     # RESAMPLING
#     p3 = []
#     index = int(random.random() * N)
#     beta = 0.0
#     mw = max(w)
#     for i in range(N):
#         beta += random.random() * 2.0 * mw
#         while beta > w[index]:
#             beta -= w[index]
#             index = (index + 1) % N
#         p3.append( particles[index] )
#     particles = p3
#     # end resampling
#
#     predicted_position = get_position(particles)
#     error = distance_between( (predicted_position[0], predicted_position[1]), (myrobot.x, myrobot.y))
#     hunter_robot.goto(predicted_position[0] * size_multiplier, predicted_position[1] * size_multiplier - 200)
#     hunter_robot.stamp()
#
#     if error <= distance_tolerance:
#         print "You got it right! It took you ", ctr, " steps to localize."
#         break
#     ctr += 1

def demo_grading_visual(estimate_next_pos_fcn, target_bot, OTHER = None):
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0

    import turtle    #You need to run this locally to use the turtle module

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
    prediction.penup()
    broken_robot.penup()


    while ctr <= 1000:
        #time.sleep(1)
        ctr += 1


        target_bot.move_in_circle()
        measurement = target_bot.sense()
        #position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)


        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)

        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            break
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."

        broken_robot.goto( target_bot.x * size_multiplier, target_bot.y * size_multiplier-200)
        broken_robot.stamp()
        prediction.goto( position_guess[0] * size_multiplier, position_guess[1] * size_multiplier-200)
        prediction.stamp()


        #End of Visualization

demo_grading_visual(estimate_next_pos, myrobot)

turtle.getscreen()._root.mainloop()




