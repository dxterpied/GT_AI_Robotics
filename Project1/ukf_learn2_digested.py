#  https://d1b10bmlvqabco.cloudfront.net/attach/io8qzl1st1xz9/hqa46wgox8l29q/iotezpnhrbck/UKF.py
import numpy as np
import scipy.stats
import turtle

"""
Given a state vector of length n, covariance matrix sigma, (non)-linear
prediction function, (non)-linear measurement function, process and
measurement noise models, and UKF parameters alpha, beta, and kappa,
perform a UKF update on the state and covariance.

Params:
    x: current state (1xn array)
    P: covariance matrix (nxn array)
    u: control command (1xn array)
    z: current measurement (1xm array)
    R: process noise
    Q: measurement noise
    process_fn: callable - given x and u, determine next state
    predict_fn: callable - given x, determine measurement
    alpha: UKF parameter (typically 1e-3 according to wikipedia)
    beta: UKF parameter (typically 2 if true distribution is gaussian)
    kappa: UKF parameter (typically 0 according to wikipedia)

Output: x, P
"""

turtle.setup(800, 800)
window = turtle.Screen()
window.bgcolor('white')

target_robot = turtle.Turtle()
target_robot.shape('turtle')
target_robot.color('green')
target_robot.shapesize(0.2, 0.2, 0.2)

predicted_robot = turtle.Turtle()
predicted_robot.shape('circle')
predicted_robot.color('blue')
predicted_robot.shapesize(0.2, 0.2, 0.2)

size_multiplier = 25




# state transition function. Used to propagate sigma points during PREDICT
# returns new matrix with updated sigma points after they are propagated through transition model
def f(sigma_points):

    #print "sigma_points", sigma_points
    # [[ 50.          50.002       50.          50.          50.          49.998     50.          50.          50.        ]
    #  [ 70.          70.          70.002       70.          70.          70.        69.998       70.          70.        ]
    #  [ 20.          20.          20.          20.002       20.          20.        20.          19.998       20.        ]
    #  [  1.57079633   1.57079633   1.57079633   1.57079633   1.57279633  1.57079633   1.57079633   1.57079633   1.56879633]]

    y = np.zeros(sigma_points.shape) # sigma_points.shape = (4, 9)

    # sigmaPointsVector:  [ 50. 70. 20. 1.57079633] - one sigma point for each variable: [x, y, v, heading]
    for index, sigmaPointsVector in enumerate(sigma_points.transpose()):
        # here only sigma point for velocity is used
        heading = sigmaPointsVector[3]
        x_velocity = np.cos(heading) * distance
        y_velocity = np.sin(heading) * distance
        F = np.array([[1, 0,  x_velocity,  0],
                      [0, 1,  y_velocity,  0],
                      [0, 0, 1, 0],   # velocity is unchanged
                      [0, 0, 0, 1]])  # heading is unchanged

        #print "F.dot(sigmaPointsVector)", F.dot(sigmaPointsVector)  # [ 50.          72.          20.           1.57079633]
        y[:, index] = F.dot(sigmaPointsVector) # multiply F by sigma points vector
        #print F.dot(sigmaPointsVector) + B.dot(u)     # [ 50.          72.          20.           1.57079633]

    #print "y", y
    # [[ 50.          50.002       50.          50.          49.996       49.998   50.          50.          50.004     ]
    #  [ 72.          72.          72.002       72.0002      71.999996    72.      71.998       71.9998      71.999996  ]
    #  [ 20.          20.          20.          20.002       20.          20.      20.          19.998       20.        ]
    #  [  1.57079633   1.57079633   1.57079633   1.57079633   1.57279633  1.57079633   1.57079633   1.57079633   1.56879633]]

    #print "r_rand.rvs( size=sigma_points.shape[1] )", r_rand.rvs( size=sigma_points.shape[1] )
    # [[-0.49333018 -0.41197113 -0.07560427 -0.04622596]
    #  [ 0.96845238  0.32706424  0.36590052 -0.10918491]
    #  [-0.57863075 -0.54928293 -0.11976924  0.0534835 ]
    #  [-0.87621096 -0.25399019 -0.06544965 -0.13573592]
    #  [-0.09652031  0.58989849  0.16892615  0.10427309]
    #  [-0.19020254  0.3664105  -0.22974149 -0.19863828]
    #  [ 0.68194096 -1.00281691  0.13785751  0.11464967]
    #  [-0.23166491  0.07741916  0.30400598  0.15220117]
    #  [-0.8766521  -0.47128929 -0.06782059 -0.16527805]]

    # why randomize here?
    y = y + r_rand.rvs( size=sigma_points.shape[1] ).transpose()

    #print "y", y
    # [[ 51.00578362  48.88448839  50.54581442  51.10090295  50.07227526    50.58837469  48.51298168  50.3024645   50.00258734]
    #  [ 71.30243751  72.48469226  71.98040602  72.78004672  72.95224762    72.95292376  72.04081463  72.44183007  72.00432578]
    #  [ 19.75970995  19.62306019  19.91618565  20.1624041   20.42853175    19.8310167   20.24519826  19.7848798   20.12271261]
    #  [  1.57927223   1.5262234    1.59228475   1.4724252    1.56111953    1.58025253   1.58077318   1.46931695   1.57668785]]


    return y



# measurement function, returns [x, y] or sigma points for x, y
def h(xk):

    #print "xk", xk

    m = xk[:2, :] + np.atleast_2d( q_rand.rvs(size=xk.shape[1] )).transpose()

    #print "m", m

    return m


def filterUsingUKF(x, P, z, R, Q, transition_fn, measurement_fn, alpha=1e-3, beta=2, kappa=0):

    L = len(x) # len is 4
    sigmaPointsCount = 2 * L + 1
    lam = alpha**2 * (L + kappa) - L
    gamma = np.sqrt(L + lam)

    sigma_root = np.linalg.cholesky(P)

    # Calculate the weights to recover the mean and covariance estimates
    weights_mean, weights_covariance = np.zeros(sigmaPointsCount), np.zeros(sigmaPointsCount)
    weights_mean[0] = gamma / (L + gamma) # weights for mean
    weights_covariance[0] = weights_mean[0] + (1. - alpha**2 + beta) # weights for covariance
    weights_mean[1:] = weights_covariance[1:] = (0.5 / (L + gamma))

    # Find the sigma points; vec[:, None] treats vec as a column to match numpy arithmetic broadcasting rules
    sigma_points = np.zeros([L, sigmaPointsCount])
    sigma_points[:, 0] = x
    sigma_points[:, 1:L+1] = x[:, None] + gamma * sigma_root
    sigma_points[:, L+1:] = x[:, None] - gamma * sigma_root

    # propagate the sigma points and control commands through the state transition function
    chi_star_bar = transition_fn(sigma_points)

    # recover the mean and covariance estimates
    mu_bar = np.sum(weights_mean * chi_star_bar, axis=1)

    sigma_bar = np.array(R)
    for w, chi_i in zip(weights_covariance, chi_star_bar.transpose()):  # use .transpose() to iterate columns
        chi_dev = (chi_i - mu_bar)[:, None]  # force into column vector
        sigma_bar += w * (chi_dev * chi_dev.T)

    # Calculate matrix sqrt of covariance by cholesky decomposition;
    # sigma_bar = L * L^* (but for Hermetian L^* = L.T). i.e.,
    # sigma_bar = sigma_bar_root.dot(cov_bar_root.T)
    sigma_bar_root = np.linalg.cholesky(sigma_bar)

    chi_bar = np.zeros([L, sigmaPointsCount])
    chi_bar[:, 0] = mu_bar
    chi_bar[:, 1:L+1] = mu_bar[:, None] + gamma * sigma_bar_root
    chi_bar[:, L+1:] = mu_bar[:, None] - gamma * sigma_bar_root


    # propagate sigma points through measurement function
    # Z_bar should have size (# observables, L)
    Z_bar = measurement_fn(chi_bar)


    z_hat = np.sum(weights_mean * Z_bar, axis=1)
    m = Z_bar.shape[0]

    S = np.array(Q)
    for w, Z_i in zip(weights_covariance, Z_bar.T):  # use .T to iterate columns
        z_dev = (Z_i - z_hat)[:, None]  # force into column vector
        S += w * (z_dev * z_dev.T)

    sigma_xz = np.zeros([L, m])
    for w, chi_bar_i, Z_i in zip(weights_covariance, chi_bar.T, Z_bar.T):
        chi_dev = (chi_bar_i - mu_bar)[:, None]  # coerce to column
        z_dev = (Z_i - z_hat)[:, None]  # coerce to column vector
        sigma_xz += w * (chi_dev * z_dev.T)

    # consider using the pseudo-inverse or catching singular errors in inv()
    K = sigma_xz.dot(np.linalg.inv(S))
    new_state = mu_bar + np.dot(K, (z - z_hat[:, None]))[0]
    new_cov = sigma_bar - np.dot(K, np.dot(S, K.T))

    return new_state, new_cov


# state vector - x, y, velocity, heading
x = np.array([ 0., 0.,  0,  0] )

P = np.matrix([[1., 0., 0., 0.], # state variance-covariance
               [0., 1., 0., 0.],
               [0., 0., 1., 0.],
               [0., 0., 0., 1.]])
R = np.diag([.5, .5, .05, 0.1])
Q = np.diag([.01] * 2)

distance = 1.5

# declare noise functions using covariances
r_rand = scipy.stats.multivariate_normal(cov=R)
q_rand = scipy.stats.multivariate_normal(cov=Q)


states = np.ndarray([100, x.shape[0]])

true_states = [[-9.021852399266194, 0.20791169081775918], [-8.108306941623594, 0.6146483338935591], [-7.2992899472486465, 1.2024335861860318], [-6.630159340889788, 1.9455784116634256], [-6.130159340889788, 2.811603815447864], [-5.821142346514841, 3.762660331743018], [-5.716613883247188, 4.757182227111291], [-5.821142346514842, 5.751704122479564], [-6.13015934088979, 6.702760638774718], [-6.630159340889791, 7.568786042559156], [-7.299289947248649, 8.31193086803655], [-8.108306941623596, 8.899716120329023], [-9.021852399266196, 9.306452763404824], [-10.000000000000002, 9.514364454222585], [-11.000000000000002, 9.514364454222587], [-11.978147600733807, 9.30645276340483], [-12.89169305837641, 8.899716120329032], [-13.70071005275136, 8.311930868036562], [-14.36984065911022, 7.568786042559171], [-14.869840659110224, 6.702760638774734], [-15.178857653485176, 5.751704122479582], [-15.283386116752833, 4.75718222711131], [-15.178857653485183, 3.762660331743036], [-14.869840659110238, 2.8116038154478815], [-14.36984065911024, 1.9455784116634418], [-13.700710052751383, 1.202433586186047], [-12.891693058376436, 0.6146483338935738], [-11.978147600733834, 0.20791169081777466], [-11.000000000000028, 1.765254609153999e-14], [-10.000000000000028, 1.9872992140790302e-14]]

#true_states = []

for i in range(100):


    z = h( x[:, None] )
    x, P = filterUsingUKF(x, P, z, R, Q, f, h)

    states[i, :] = x.copy()

#print true_states
#print states


for i in true_states:
    target_robot.goto(i[0] * size_multiplier, i[1] * size_multiplier)
    target_robot.stamp()

for i in states:
    predicted_robot.goto(i[0] * 10, i[1] * 10)
    predicted_robot.stamp()

turtle.getscreen()._root.mainloop()



