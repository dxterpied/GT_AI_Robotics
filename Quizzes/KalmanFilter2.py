from math import *

# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.

def update(oldMean, oldVariance, newMean, newVariance):
    new_mean = ( newVariance * oldMean + oldVariance * newMean ) / (oldVariance + newVariance)
    new_var = 1./ ( 1./oldVariance + 1./newVariance  )
    return [new_mean, new_var]

print update(10., 8., 13., 2.)
