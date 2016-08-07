from math import *

# for i in range(100):
#     execfile('RR2.py')


# both atan2 and asin give the same angles as they should
# print "atan2(4.0, 3.0): ", atan2(4.0, 3.0)
# print "asin(4.0/5.0): ", asin(4.0/5.0)
# atan2(4.0, 3.0):  0.927295218002
# asin(4.0/5.0):  0.927295218002


# print atan(1)      # 0.785398163397
# print atan2(1, 1)  # 0.785398163397
# print atan2(-1, -1) # -2.35619449019
#print atan2(0, -1)   # 3.14159265359

print atan2(1, 0)                           # 1.57079632679
print degrees(atan2(1, 0)) % 360            # 90.0
print radians(degrees(atan2(1, 0)) % 360)   # 1.57079632679

# negative angle conversion to positive
print atan2(-1, 0)                          # -1.57079632679
print degrees(atan2(-1, 0)) % 360           # 270.0
print radians(degrees(atan2(-1, 0)) % 360)  # 4.71238898038




# def mult(x, y):
#     return x * y
# print map(mult, [3, 2, 0], [1,  4, 1]) # returns [3, 8, 0]

# 1 radian	=	180/pi
# To go from radians to degrees: multiply by 180, divide by pi
# To go from degrees to radians: multiply by pi, divide by 180

print 1.5 / (sin(pi/30.) * 2.)
print "1.5 / (2. * sin(pi/30.))", 1.5 / (2. * sin(pi/30.))


