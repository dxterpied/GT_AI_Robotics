from math import *

# Kalman filter

# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.


# update when sensing new data
def update(oldEstimate, errEstimate, measurement, errMeasurement):
    new_estimate = ( errMeasurement * oldEstimate + errEstimate * measurement ) / (errEstimate + errMeasurement)
    # the sucker above already contains Kalman gain!
    #new_mean = (errEstimate/(errEstimate + errMeasurement)) * measurement + (errEstimate/(errEstimate + errMeasurement)) * oldEstimate
    # new_mean = Weight 1 * measurement + Weight 2 * oldEstimate
    # Weight 1 + Weight 2 = 1
    # errEstimate/(errEstimate + errMeasurement) + errEstimate/(errEstimate + errMeasurement) = 1
    # so, the Kalman Gain = errEstimate/(errEstimate + errMeasurement)
    # new_mean = KG * measurement + (1 - KG) * oldEstimate = oldEstimate + KG * (measurement - oldEstimate)

    new_estimate_error = (errEstimate * errMeasurement) / (errEstimate + errMeasurement)
    return [new_estimate, new_estimate_error]

# predict
def predict(oldEstimate, errEstimate, measurement, errMeasurement):
    new_estimate = oldEstimate + measurement
    new_estimate_error = errEstimate + errMeasurement
    return [new_estimate, new_estimate_error]


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



