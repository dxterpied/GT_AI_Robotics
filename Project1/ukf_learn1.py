'''
==============================================
Using the Unscented Kalman Filter and Smoother
==============================================

This simple example shows how one may apply the Unscented Kalman Filter and
Unscented Kalman Smoother to some randomly generated data.

The Unscented Kalman Filter (UKF) and Rauch-Rung-Striebel type Unscented Kalman
Smoother (UKS) are a generalization of the traditional Kalman Filter and
Smoother to models with non-linear equations describing state transitions and
observation emissions. Unlike the Extended Kalman Filter (EKF), which attempts
to perform the same task by using the numerical derivative of the appropriate
equations, the UKF selects a handful of "sigma points", passes them through the
appropriate function, then finally re-estimates a normal distribution around
those propagated points. Experiments have shown that the UKF and UKS are
superior to the EKF and EKS in nearly all scenarios.

The figure drawn shows the true, hidden state; the state estimates given by the
UKF; and finally the same given by the UKS.
'''
import numpy as np
import pylab as pl
from pykalman import UnscentedKalmanFilter

# F matrix - next state function
def transition_function(state, noise):
    a = np.sin(state[0]) + state[1] * noise[0]
    b = state[1] + noise[1]
    return np.array([a, b])

# H matrix - measurement function
def observation_function(state, noise):
    C = np.array([[-1, 0.5], [0.2, 0.1]])
    return np.dot(C, state) + noise


transition_covariance = np.eye(2)  # identity matrix
# [[ 1.  0.]
#  [ 0.  1.]]

random_state = np.random.RandomState(0)
measurement_uncertainty = np.eye(2) + random_state.randn(2, 2) * 0.1  # R matrix;  measurement uncertainty
# [[ 1.17640523  0.04001572]
#  [ 0.0978738   1.22408932]]

initial_state_mean = [0, 0] # x matrix  # state matrix

initial_state_covariance = [[1, 0.1],
                            [-0.1, 1]]

# sample from model
kf = UnscentedKalmanFilter(
    transition_function, observation_function,
    transition_covariance, measurement_uncertainty,
    initial_state_mean, initial_state_covariance,
    random_state=random_state
)

states = [[0., 0.], [2. ,2.], [5., 5.]]
observations = [[0.1, 0.2], [2.1 ,2.2], [5.0,5.1]]
states, observations = kf.sample(50, initial_state_mean)

#print states, observations

# estimate state with filtering and smoothing
filtered_state_estimates = kf.filter(observations)[0]

print filtered_state_estimates

# draw estimates
pl.figure()
lines_true = pl.plot(states, color='b')
lines_filt = pl.plot(filtered_state_estimates, color='r', ls='-')


pl.legend((lines_true[0], lines_filt[0]),
          ('actual data', 'UKF'),
          loc='lower left')


pl.show()
