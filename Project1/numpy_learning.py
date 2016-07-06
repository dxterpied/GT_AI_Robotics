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

print "arithmetic multiplication: "
print A * B