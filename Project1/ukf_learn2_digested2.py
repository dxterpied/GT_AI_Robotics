# https://d1b10bmlvqabco.cloudfront.net/attach/io8qzl1st1xz9/hqa46wgox8l29q/iotezpnhrbck/UKF.py


import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import multivariate_normal as mv_norm


"""
Given a state vector of length n, covariance matrix sigma, (non)-linear
prediction function, (non)-linear measurement function, process and
measurement noise models, and UKF parameters alpha, beta, and kappa,
perform a UKF update on the state and covariance.

Params:
---
    state_mu: current state (1xn array)

    cov_sigma: covariance matrix (nxn array)

    control_u: control command (1xn array)

    measurement_z: current measurement (1xm array)

    process_fn: callable - given x and u, determine next state

    predict_fn: callable - given x, determine measurement

    noise_Rt: process noise

    noise_Qt: measurement noise

    alpha: UKF parameter (typically 1e-3 according to wikipedia)

    beta: UKF parameter (typically 2 if true distribution is gaussian)

    kappa: UKF parameter (typically 0 according to wikipedia)

Output:
---
    new_state

    new_cov
"""


def UKF(state_mu, cov_sigma, control_u, measurement_z, process_fn,
        predict_fn, noise_Rt, noise_Qt, alpha=1e-3, beta=2, kappa=0):

    n = len(state_mu)
    L = 2 * n + 1
    lam = alpha**2 * (n + kappa) - n
    gamma = np.sqrt(n + lam)

    # Calculate matrix sqrt of covariance by cholesky decomposition;
    # sigma = L * L^* (but for Hermetian L^* = L.T). i.e.,
    # sigma = sigma_root.dot(sigma_root.T)
    sigma_root = np.linalg.cholesky(cov_sigma)

    # Calculate the weights to recover the mean and covariance estimates
    wm, wc = np.zeros(L), np.zeros(L)
    wm[0] = gamma / (n + gamma)
    wc[0] = wm[0] + (1. - alpha**2 + beta)
    wm[1:] = wc[1:] = (0.5 / (n + gamma))

    # Find the sigma points; vec[:, np.newaxis] treats vec as a column
    # to match numpy arithmetic broadcasting rules
    chi = np.zeros([n, L])
    chi[:, 0] = state_mu
    chi[:, 1:n+1] = state_mu[:, np.newaxis] + gamma * sigma_root
    chi[:, n+1:] = state_mu[:, np.newaxis] - gamma * sigma_root

    # propagate the sigma points and control commands through the process model
    chi_star_bar = process_fn(control_u, chi)

    # recover the mean and covariance estimates
    mu_bar = np.sum(wm * chi_star_bar, axis=1)

    sigma_bar = np.array(noise_Rt)
    for w, chi_i in zip(wc, chi_star_bar.T):  # use .T to iterate columns
        chi_dev = (chi_i - mu_bar)[:, np.newaxis]  # force into column vector
        sigma_bar += w * (chi_dev * chi_dev.T)

    # Calculate matrix sqrt of covariance by cholesky decomposition;
    # sigma_bar = L * L^* (but for Hermetian L^* = L.T). i.e.,
    # sigma_bar = sigma_bar_root.dot(cov_bar_root.T)
    sigma_bar_root = np.linalg.cholesky(sigma_bar)

    chi_bar = np.zeros([n, L])
    chi_bar[:, 0] = mu_bar
    chi_bar[:, 1:n+1] = mu_bar[:, np.newaxis] + gamma * sigma_bar_root
    chi_bar[:, n+1:] = mu_bar[:, np.newaxis] - gamma * sigma_bar_root

    # Z_bar should have size (# observables, L)
    Z_bar = predict_fn(chi_bar)
    z_hat = np.sum(wm * Z_bar, axis=1)
    m = Z_bar.shape[0]

    S = np.array(noise_Qt)
    for w, Z_i in zip(wc, Z_bar.T):  # use .T to iterate columns
        z_dev = (Z_i - z_hat)[:, np.newaxis]  # force into column vector
        S += w * (z_dev * z_dev.T)

    sigma_xz = np.zeros([n, m])
    for w, chi_bar_i, Z_i in zip(wc, chi_bar.T, Z_bar.T):
        chi_dev = (chi_bar_i - mu_bar)[:, np.newaxis]  # coerce to column
        z_dev = (Z_i - z_hat)[:, np.newaxis]  # coerce to column vector
        sigma_xz += w * (chi_dev * z_dev.T)

    # consider using the pseudo-inverse or catching singular errors in inv()
    K = sigma_xz.dot(np.linalg.inv(S))
    new_state = mu_bar + np.dot(K, (measurement_z - z_hat[:, np.newaxis]))[0]
    new_cov = sigma_bar - np.dot(K, np.dot(S, K.T))

    return new_state, new_cov


def main():

    # initial state vector - position (xy), V, heading
    state = np.array([0., 0.])
    cov = np.eye(2)
    dt = 0.1
    R = np.diag([.5, .5])
    Q = np.diag([.01, .01])

    # declare noise functions using covariances
    r_rand = mv_norm(cov=R)
    q_rand = mv_norm(cov=Q)


    def g(uk, xk):
        # an arbitrary nonlinear function from x to y
        y = np.zeros(xk.shape)
        for i, x_i in enumerate(xk.T):


            A = np.array([[1., 0.]])


            B = np.array([[0., 1.]])

            y[:, i] = A.dot(x_i) + B.dot(uk)

        eps = r_rand.rvs(size=xk.shape[1]).T
        y += eps
        # print y
        # [[-0.34425082  0.52426215  0.91768908 -0.54485885 -0.49293042]
        # [-1.26491096  0.58948238 -0.03959105 -0.06719506  0.54006089]]
        return y



    def h(xk):
        # measure function - predict the measurements given xk
        # only measure location
        m = xk[:1, :] + np.atleast_2d(q_rand.rvs(size=xk.shape[1])).T
        return m

    # apply the
    controls = {10: np.array([0., np.pi / 6]),
                30: np.array([-.5, 0.]),
                50: np.array([0., -np.pi / 6]),
                60: np.array([0., -np.pi / 6]),
                70: np.array([1., 0.]),
                85: np.array([-.5, np.pi / 6]),
                100: np.array([0., 0.])
                }

    states = np.ndarray([100, state.shape[0]])
    u = np.array([0., 0.])


    for i in range(100):
        u += controls.get(i, np.array([0., 0.])) * dt
        state, cov = UKF(state, cov, u, h(state[:, np.newaxis]), g, h, R, Q)
        states[i, :] = state.copy()
        # print u

    fig = plt.figure()
    ax1 = fig.add_subplot(311)
    ax1.plot(states[:, 0], color='b')
    ax1.plot(states[:, 1], color='r')

    ax1.set_title('x-y position')

    # ax2 = fig.add_subplot(312)
    # ax2.plot(states[:, 2])
    # ax2.set_title('Speed')
    #
    # ax3 = fig.add_subplot(313)
    # ax3.plot(states[:, 3])
    # ax3.set_title('Heading')

    plt.show()



if __name__ == "__main__":
    main()