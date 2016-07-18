from math import *

# Kalman filter

# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.


# update is used for sensing
def update(oldMean, oldVariance, newMean, newVariance):
    new_mean = ( newVariance * oldMean + oldVariance * newMean ) / (oldVariance + newVariance)

    new_var = 1./ ( 1./oldVariance + 1./newVariance  )

    return [new_mean, new_var]

# this is used for motion prediction
def predict(oldMean, oldVariance, newMean, newVariance):
    new_mean = oldMean + newMean
    new_var = oldVariance + newVariance
    return [new_mean, new_var]


measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]

measurementVariance = 4.
motionVariance = 2.
mu = 0. # initial value that will be updated
sigma = 10000. # initial value that will be updated

# Kalman filter
for i in range(len(measurements)):

    # predict
    [mu, sigma] = predict(mu, sigma, motion[i], motionVariance)
    print 'predict ', [mu, sigma]

    # update
    [mu, sigma] = update(mu, sigma, measurements[i], measurementVariance)
    print 'update ', [mu, sigma]



print "Final result:"
print [mu, sigma]



