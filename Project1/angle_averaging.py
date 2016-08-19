import numpy as np
import turtle
from cmath import rect, phase
from math import radians, degrees, atan2


size_multiplier = 100.

turtle.setup(450, 450, 0, 600)

actual_angles = turtle.Turtle()
actual_angles.shape('circle')
actual_angles.color('black')
actual_angles.penup()
actual_angles.shapesize(0.3, 0.3, 0.3)

circle = turtle.Turtle()
circle.shape('circle')
circle.color('orange')
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


for i in range(360):
    angle = radians(i)
    x = np.cos(angle)
    y = np.sin(angle)
    circle.goto(x * size_multiplier, y * size_multiplier)
    circle.stamp()

    if i % 90 == 0:
        actual_angles.goto( x * size_multiplier, y * size_multiplier)
        actual_angles.stamp()


faulty_angles = [np.pi * 0.1, np.pi * 0.75, np.pi * 1.1, np.pi * 1.4]
faulty_xy = [(np.cos(faulty_angles[0]), np.sin(faulty_angles[0])),
             (np.cos(faulty_angles[1]), np.sin(faulty_angles[1])),
             (np.cos(faulty_angles[2]), np.sin(faulty_angles[2])),
             (np.cos(faulty_angles[3]), np.sin(faulty_angles[3]))]


faulty_angles_turtle.goto( faulty_xy[0][0] * size_multiplier, faulty_xy[0][1] * size_multiplier)
faulty_angles_turtle.stamp()
faulty_angles_turtle.goto( faulty_xy[1][0] * size_multiplier, faulty_xy[1][1] * size_multiplier)
faulty_angles_turtle.stamp()
faulty_angles_turtle.goto( faulty_xy[2][0] * size_multiplier, faulty_xy[2][1] * size_multiplier)
faulty_angles_turtle.stamp()
faulty_angles_turtle.goto( faulty_xy[3][0] * size_multiplier, faulty_xy[3][1] * size_multiplier)
faulty_angles_turtle.stamp()


average_x = np.mean(zip(*faulty_xy)[0])
average_y = np.mean(zip(*faulty_xy)[1])

average_turtle.goto( np.cos(atan2(average_y, average_x )) * size_multiplier, np.sin(atan2(average_y, average_x )) * size_multiplier)
average_turtle.stamp()

average_angle_turtle.goto( np.cos(np.mean(faulty_angles)) * size_multiplier, np.sin(np.mean(faulty_angles)) * size_multiplier)
average_angle_turtle.stamp()

#print zip(*faulty_xy)

print faulty_angles

# myTurtle = turtle.Turtle(shape="turtle")
# myTurtle.penup()
# myTurtle.setposition(0,0)
# myTurtle.pendown()
# myTurtle.circle(50)

turtle.getscreen()._root.mainloop()



