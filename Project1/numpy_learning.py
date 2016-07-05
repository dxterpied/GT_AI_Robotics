import numpy as np

# multidimensional array

A = np.array([[1, 2],
               [3, 4]])
print(A)
B = A.transpose()
print B

B = np.array([[5, 6],
               [7, 8]])
print B
print A * B
