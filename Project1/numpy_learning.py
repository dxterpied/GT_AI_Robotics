import numpy as np

# multidimensional array

A = np.array([[1, 2],
               [3, 4]])
print(A)

B = A.transpose()

B = np.array([[5, 6],
               [7, 8]])
print B

print "dot product (matrix multiplication): "
print np.dot(A, B)
# [[19 22]
#  [43 50]]

print "arithmetic multiplication: "
print A * B
# [[ 5 12]
#  [21 32]]

# if you do np.matrix(), you can multiply using *