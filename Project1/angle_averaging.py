import numpy as np
import turtle
from cmath import rect, phase
from math import radians, degrees, atan2

x_actual = np.array([0.0, 1.4672214011007085, 2.83753958756461, 4.0510650791270315, 5.054760988665319, 5.80476098866532, 6.268286480227742, 6.4250791751292216, 6.268286480227741, 5.804760988665318, 5.054760988665317, 4.051065079127029, 2.837539587564608, 1.4672214011007072, -8.881784197001252e-16, -1.5000000000000009, -2.9672214011007103, -4.337539587564613, -5.551065079127038, -6.554760988665329, -7.304760988665335, -7.768286480227762, -7.925079175129248, -7.768286480227774, -7.3047609886653575, -6.55476098866536, -5.551065079127074, -4.3375395875646525, -2.9672214011007503, -1.500000000000041, -4.107825191113079e-14, 1.4672214011006668, 2.837539587564567, 4.051065079126986, 5.054760988665272, 5.8047609886652705, 6.268286480227689, 6.4250791751291665, 6.268286480227683, 5.804760988665258, 5.0547609886652545, 4.051065079126965, 2.8375395875645424, 1.4672214011006404, -6.816769371198461e-14, -1.5000000000000682, -2.967221401100777, -4.337539587564679, -5.551065079127102, -6.554760988665391, -7.304760988665395, -7.76828648022782, -7.925079175129303, -7.768286480227826, -7.304760988665407, -6.55476098866541, -5.551065079127124, -4.337539587564702, -2.9672214011008, -1.5000000000000908, -9.08162434143378e-14, 1.467221401100617, 2.837539587564517, 4.0510650791269365, 5.0547609886652225, 5.804760988665221, 6.26828648022764, 6.425079175129117, 6.2682864802276335, 5.804760988665208, 5.054760988665205, 4.051065079126915, 2.8375395875644926, 1.4672214011005906, -1.1790568521519162e-13, -1.500000000000118, -2.9672214011008267, -4.337539587564729, -5.551065079127151, -6.554760988665441, -7.3047609886654445, -7.76828648022787, -7.925079175129353, -7.768286480227876, -7.304760988665457, -6.55476098866546, -5.551065079127174, -4.337539587564752, -2.9672214011008498, -1.5000000000001406, -1.4055423491754482e-13])
y_actual = np.array([10.0, 10.31186753622664, 10.92197250084034, 11.803650379279048, 12.91836761749514, 14.217405723171797, 15.643990497614528, 17.135773340666937, 18.627556183719346, 20.054140958162076, 21.353179063838734, 22.467896302054825, 23.349574180493534, 23.959679145107234, 24.271546681333874, 24.271546681333877, 23.95967914510724, 23.349574180493544, 22.46789630205484, 21.353179063838752, 20.054140958162098, 18.62755618371937, 17.13577334066696, 15.643990497614551, 14.217405723171819, 12.918367617495159, 11.803650379279066, 10.921972500840356, 10.311867536226657, 10.000000000000021, 10.000000000000025, 10.311867536226666, 10.921972500840369, 11.80365037927908, 12.918367617495173, 14.217405723171833, 15.643990497614563, 17.135773340666972, 18.62755618371938, 20.05414095816211, 21.353179063838766, 22.467896302054854, 23.349574180493562, 23.95967914510726, 24.2715466813339, 24.2715466813339, 23.95967914510726, 23.349574180493562, 22.467896302054854, 21.353179063838766, 20.05414095816211, 18.62755618371938, 17.135773340666972, 15.643990497614562, 14.217405723171831, 12.918367617495171, 11.803650379279079, 10.921972500840369, 10.31186753622667, 10.000000000000034, 10.000000000000037, 10.311867536226679, 10.921972500840381, 11.803650379279093, 12.918367617495186, 14.217405723171845, 15.643990497614576, 17.135773340666987, 18.627556183719395, 20.054140958162126, 21.35317906383878, 22.467896302054868, 23.349574180493576, 23.959679145107273, 24.271546681333913, 24.271546681333913, 23.959679145107273, 23.349574180493576, 22.467896302054868, 21.35317906383878, 20.054140958162126, 18.627556183719395, 17.135773340666987, 15.643990497614576, 14.217405723171845, 12.918367617495186, 11.803650379279093, 10.921972500840383, 10.311867536226684, 10.000000000000048, 10.000000000000052])



# takes degrees in radians and outputs radians
def mean_angle(deg):
    return phase( sum(rect(1, d) for d in deg) / len(deg) )


size_multiplier = 150.
angles = []

turtle.setup(450, 450, 400, 200)

actual_angles = turtle.Turtle()
actual_angles.shape('circle')
actual_angles.color('black')
actual_angles.penup()
actual_angles.shapesize(0.3, 0.3, 0.3)

circle = turtle.Turtle()
circle.shape('circle')
circle.color('bisque2')
circle.shapesize(0.05, 0.05, 0.05)

faulty_angles_turtle = turtle.Turtle()
faulty_angles_turtle.shape('circle')
faulty_angles_turtle.color('red')
faulty_angles_turtle.penup()
faulty_angles_turtle.shapesize(0.2, 0.2, 0.2)

average_turtle = turtle.Turtle()
average_turtle.shape('circle')
average_turtle.color('blue')
average_turtle.penup()
average_turtle.shapesize(0.3, 0.3, 0.3)

average_angle_turtle = turtle.Turtle()
average_angle_turtle.shape('square')
average_angle_turtle.color('green')
average_angle_turtle.penup()
average_angle_turtle.shapesize(0.3, 0.3, 0.3)

actual_angle_turtle = turtle.Turtle()
actual_angle_turtle.shape('square')
actual_angle_turtle.color('brown')
actual_angle_turtle.penup()
actual_angle_turtle.shapesize(0.3, 0.3, 0.3)



for i in range(1080):
    angle = radians(i)
    x = np.cos(angle)
    y = np.sin(angle)

    circle.goto(x * size_multiplier, y * size_multiplier)
    circle.stamp()

    if i % 12 == 0:
        theta = atan2(y, x)
        angles.append(theta)

        actual_angle_turtle.goto( np.cos(theta) * size_multiplier, np.sin(theta) * size_multiplier)
        actual_angle_turtle.stamp()

    if i % 90 == 0:
        actual_angles.goto( x * size_multiplier, y * size_multiplier)
        actual_angles.stamp()


faulty_angles = [np.pi * 0.1, np.pi * 0.75, np.pi * 1.1, np.pi * 1.4]
faulty_xy = [(np.cos(faulty_angles[0]), np.sin(faulty_angles[0])),
             (np.cos(faulty_angles[1]), np.sin(faulty_angles[1])),
             (np.cos(faulty_angles[2]), np.sin(faulty_angles[2])),
             (np.cos(faulty_angles[3]), np.sin(faulty_angles[3]))]


# faulty_angles_turtle.goto( faulty_xy[0][0] * size_multiplier, faulty_xy[0][1] * size_multiplier)
# faulty_angles_turtle.stamp()
# faulty_angles_turtle.goto( faulty_xy[1][0] * size_multiplier, faulty_xy[1][1] * size_multiplier)
# faulty_angles_turtle.stamp()
# faulty_angles_turtle.goto( faulty_xy[2][0] * size_multiplier, faulty_xy[2][1] * size_multiplier)
# faulty_angles_turtle.stamp()
# faulty_angles_turtle.goto( faulty_xy[3][0] * size_multiplier, faulty_xy[3][1] * size_multiplier)
# faulty_angles_turtle.stamp()


average_x = np.mean(zip(*faulty_xy)[0])
average_y = np.mean(zip(*faulty_xy)[1])

# average_turtle.goto( np.cos(atan2(average_y, average_x )) * size_multiplier, np.sin(atan2(average_y, average_x )) * size_multiplier)
# average_turtle.stamp()

average_angle_turtle.goto( np.cos(mean_angle(angles)) * size_multiplier, np.sin(mean_angle(angles)) * size_multiplier)
average_angle_turtle.stamp()

#print zip(*faulty_xy)

# print faulty_angles

# myTurtle = turtle.Turtle(shape="turtle")
# myTurtle.penup()
# myTurtle.setposition(0,0)
# myTurtle.pendown()
# myTurtle.circle(50)


# for angles in [[ radians(-20), radians(10)], faulty_angles]:
#     print 'The mean angle of', angles, 'is:', round(mean_angle(angles), 12), 'radians'


# for i in range(0, 30):
#     angles.append(atan2( y_actual[i], x_actual[i] ))
#
#     average_angle_turtle.goto( np.cos(angles[i]) * size_multiplier, np.sin(angles[i]) * size_multiplier)
#     average_angle_turtle.stamp()


print "mean_angle(angles)", mean_angle(angles)
#print "mean(angles)", np.mean(angles)
#print angles


turtle.getscreen()._root.mainloop()



