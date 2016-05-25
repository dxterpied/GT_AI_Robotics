from math import *

# Kalman filter

# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.


# update is used for sensing
def update(oldMean, oldVariance, newMean, measurement_sig):
    new_mean = ( measurement_sig * oldMean + oldVariance * newMean ) / (oldVariance + measurement_sig)
    new_var = 1./ ( 1./oldVariance + 1./measurement_sig  )
    return [new_mean, new_var]

# this is used for motion prediction
def predict(oldMean, oldVariance, newMean, motion_sig):
    new_mean = oldMean + newMean
    new_var = oldVariance + motion_sig
    return [new_mean, new_var]


measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]

measurement_sig = 4.
motion_sig = 2.
mu = 0. # initial value that will be updated
sig = 10000. # initial value that will be updated

# Kalman filter
for i in range(len(measurements)):
    [mu, sig] = update(mu, sig, measurements[i], measurement_sig)
    #print 'update ', [mu, sig]

    [mu, sig] = predict(mu, sig, motion[i], motion_sig)
    #print 'predict ', [mu, sig]



print [mu, sig]



