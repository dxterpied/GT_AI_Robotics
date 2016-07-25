import numpy as np
from numpy.random import multivariate_normal
from filterpy.kalman import unscented_transform
from filterpy.kalman import MerweScaledSigmaPoints as SigmaPoints
import scipy.stats as stats
import matplotlib.pyplot as plt


def f_nonlinear_xy(x, y):
    return np.array([x + y, .1*x**2 + y*y])

mean = (0, 0)
p = np.array([[32, 15], [15., 40.]])
# Compute linearized mean
mean_fx = f_nonlinear_xy(*mean)

#generate random points
xs, ys = multivariate_normal(mean=mean, cov=p, size=10000).T


#initial mean and covariance
mean = (0, 0)
p = np.array([[32., 15], [15., 40.]])

# create sigma points - we will learn about this later
points = SigmaPoints(n=2, alpha=.1, beta=2., kappa=1.)
Wm, Wc = points.weights()
sigmas = points.sigma_points(mean, p)

### pass through nonlinear function
sigmas_f = np.empty((5, 2))
for i in range(5):
    sigmas_f[i] = f_nonlinear_xy(sigmas[i, 0], sigmas[i ,1])

### use unscented transform to get new mean and covariance
ukf_mean, ukf_cov = unscented_transform(sigmas_f, Wm, Wc)

#generate random points
np.random.seed(100)
xs, ys = multivariate_normal(mean=mean, cov=p, size=5000).T

plt.figure()
#plot_monte_carlo_mean(xs, ys, f_nonlinear_xy, ukf_mean, 'Unscented Mean')
plt.xlim(-30, 30); plt.ylim(0, 90)
plt.subplot(121)
plt.scatter(sigmas[:,0], sigmas[:,1], c='r', s=30);
plt.show()


