from math import *

# for i in range(100):
#     execfile('RR2.py')

#check 3, 4, 5, triangle

# both atan2 and asin give the same angles as they should
# print "atan2(4.0, 3.0): ", atan2(4.0, 3.0)
# print "asin(4.0/5.0): ", asin(4.0/5.0)
# atan2(4.0, 3.0):  0.927295218002
# asin(4.0/5.0):  0.927295218002


# this
# print atan(1)
# print atan2(1, 1)
# print atan2(-1, -1)

# prints out
# 0.785398163397
# 0.785398163397
# -2.35619449019

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# headingAngle2 1.82823564346
# headingAngleAvg2 1.31335701013
# point2 (5.557196583339297, 12.0678036059967)
# point3 (5.200397287070527, 13.423004363874607)
# x2Delta -0.356799296269 -0.356799296269
# y2Delta 1.35520075788  1.35520075788
# hypotenuse2 1.40138318527

#headingAngle2 = atan2(y2Delta, x2Delta): 1.82823564346, correct
print atan2(1.35520075788, -0.356799296269)
#  headingAngleAvg2 = asin(y2Delta / hypotenuse2): 1.31335701014, correct
print  asin(1.35520075788 / 1.40138318527)
# y2Delta = point3[1] - point2[1]: 1.35520075788: correct
print 13.423004363874607 - 12.0678036059967
# x2Delta = point3[0] - point2[0]: -0.356799296269, correct
print 5.200397287070527 - 5.557196583339297
print distance_between((5.557196583339297, 12.0678036059967), (5.200397287070527, 13.423004363874607))
print sqrt(1.9721)
