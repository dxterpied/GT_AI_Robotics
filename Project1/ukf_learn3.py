# This code is subject under the terms of the BSD License
# https://geus.wordpress.com/2011/01/17/monodimensional-ukf-unscented-kalman-filter-in-python/

import scipy.stats
from numpy import *
from matplotlib.pyplot import *

def getSigmaPoints(mu, sigma, n=1., alpha=1., kappa=1., betta=2.):
 #space for sigma points
 X = zeros(2 * n + 1)
 wm = zeros(2 * n + 1)
 wc = zeros(2 * n + 1)
 lambd = alpha * alpha * (n + kappa) - n

 #calculate sigma points
 X[0] = mu
 X[1] = mu + (math.sqrt((n + lambd) * sigma))
 X[2] = mu - (math.sqrt((n + lambd) * sigma))

 #calculate weights
 #first weight
 wm[0] = lambd / (n + lambd)
 wc[0] = lambd / (n + lambd) + (1 - alpha * alpha + betta)

 #rest of weights
 wm[1] = wc[1] = 1 / (2. * (n + lambd))
 wm[2] = wc[2] = 1 / (2. * (n + lambd))

 return [X, wm, wc]

def process_state_transition(x, u):
 return u +  x*x

def measurement(x):
 return x;

def measurement_noise(measurement_cov):
 return scipy.stats.norm.rvs(loc=0, scale=measurement_cov, size=1)

def process_noise(process_cov):
 return scipy.stats.norm.rvs(loc=0, scale=process_cov, size=1)

def h(x):
 return x;

#UKF parameters
alpha = 1.;
kappa = 0.;
betta = 2.;
n = 1;

#model parameters
#process noise covariance matrix
R = 0.45
Q = 0.3

#suppose the control input u = 1
u = 2;

#previous state believe
mu_prev = 1;
sigma_prev = 0.1;

real_prior_state = scipy.stats.norm.rvs(loc=mu_prev, scale=sigma_prev, size=1)
print "real previous state %f"%(real_prior_state)
print "posterior belief of the previous state (mu=%f,sigma=%f)"%(mu_prev,sigma_prev)

f = scipy.stats.norm(mu_prev, sigma_prev)
#getting sigma points from previous state belief
[X_prior_state, wm, wc] = getSigmaPoints(mu_prev, sigma_prev, n, alpha, kappa, betta)

#plot this previous state belief
#http://matplotlib.sourceforge.net/users/pyplot_tutorial.html
abcises = linspace(-1, 5, 400)
pX = f.pdf(X_prior_state)
figure(1)
subplot(211)
#plot gaussian function
plot(abcises, f.pdf(abcises),"b")
#plot sigma points
plot(X_prior_state, pX, "r+")
show();

#plot real previous state
axvline(real_prior_state,color="r")

print "...process happening..."
real_current_state = process_state_transition(real_prior_state,u) +  process_noise(R);
print "real current state %f"%(real_current_state)

#applying the process_state_transition to all sigma points to later build the prior belief
X_current_state = [process_state_transition(x, u) for x in X_prior_state]
plot(X_current_state, pX, "b+")

#rebuild the gaussian function after applying the process state transition to the sigma points samples
prior_bel_mu = sum(wm*X_current_state);
prior_bel_sigma_without_noise = sum (wc * (X_current_state - prior_bel_mu)* (X_current_state - prior_bel_mu))
prior_bel_sigma = prior_bel_sigma_without_noise + R

prior_bel_f_without_noise = scipy.stats.norm(prior_bel_mu, prior_bel_sigma_without_noise)
prior_bel_f = scipy.stats.norm(prior_bel_mu, prior_bel_sigma)
print "prior belief (mu=%f,sigma=%f)"%(prior_bel_mu,prior_bel_sigma)

#plot the prior belief after the process state transition without noise
#plot(abcises, prior_bel_f_without_noise.pdf(abcises),"g")
#plot the prior belief (with noise)
plot(abcises, prior_bel_f.pdf(abcises),"y")
#plot real current state
axvline(real_current_state,color="r")
subplot(212)
#also plot it in the bottom frame
axvline(real_current_state,color="r")

#so now our best guess is the prior belief. Then we get its sigma points
[X_prior_bel, wm, wc] = getSigmaPoints(prior_bel_mu, prior_bel_sigma, n, alpha, kappa, betta)

#so given the sigma points hypotesys, we get the expected measurements given our measurement model and noise
#in some way this is a substate of the prior_belief
Z_expected = [h(x) for x in X_prior_bel]
z_mu_expected = sum(wm*Z_expected);
z_sigma_expected_without_noise = sum (wc * (Z_expected - z_mu_expected)* (Z_expected - z_mu_expected))
z_sigma_expected = z_sigma_expected_without_noise + Q;
print "expected measurement (mu=%f,sigma=%f)"%(z_mu_expected,z_sigma_expected)

#plot the expected measurement given our best state guess (prior_belief)
#observe how the quality in this expectation depends on the prior belief convariance which
#at the same time depends on process covariance R
#In any case the expected measurement can be "more precise" or "less precise" that the prior belief


plot(abcises, scipy.stats.norm(z_mu_expected,z_sigma_expected).pdf(abcises),"b")

# calculate the gain, who we trust: process or measurement
K = sum(wc*(X_prior_bel - prior_bel_mu)*(Z_expected - z_mu_expected))/z_sigma_expected
print "Gain: %f"%K

# if K gets 1 trust the measurement
# if K gets 0 trust the process state transition model

#simulate observation
z = measurement(real_current_state) + measurement_noise(Q);
print "measurement %f"%(z)
plot(abcises, scipy.stats.norm(z,Q).pdf(abcises),"r")

#calculate the posterior gain
post_bel_mu = prior_bel_mu + K * h(z - z_mu_expected)
post_bel_sigma = prior_bel_sigma - K * z_sigma_expected *K

print "posterior belef (mu=%f,sigma=%f)"%(post_bel_mu,post_bel_sigma)
plot(abcises, scipy.stats.norm(post_bel_mu,post_bel_sigma).pdf(abcises),"g--")