from robot import *
from matrix import *
import turtle
from numpy import zeros, eye, diag, sin, cos, linalg, pi, matrix

# RR4_EKF ----------------------------------------------------------------------------------------
def EKF_Predict(X = None, P = None, dt = 0.):
    # Extended Kalman Filter Motion Estimate for nonlinear X state modeled with constant velocity and angular velocity
    max_speed = 1.5 # taken from problem in this case
    max_turn_rate = pi/4 # max of 45deg/sec
    # Various motion noise for Q
    x_var = y_var = max_speed    # set for max speed
    heading_var = max_turn_rate    # Assuming max turn in a step
    v_var = max_speed               # set for max speed
    turning_var = max_turn_rate     # assuming low acceleration
    if type(X) == type(None): # Initialize X statespace
        X = matrix([[0.],  # x
                    [0.],  # y
                    [0.],  # heading (x_dir is 0 deg, y_dir is 90 deg)
                    [0.],  # velocity
                    [0.]]) # turning angle (positive is counter clockwise)
    if type(P) == type(None): # Initialize Uncertainty Matrix - no correlation uncertainty
        P = diag([1000., 1000., 2*pi, 100., 2*pi])
    x, y, heading, v, turning = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0]
    if abs(turning) < 0.0001: # Avoid divide by zero, use as if no turning
        # Using a linear FX for this case of no turning
        FX = matrix([[x + v * cos(heading)],   # basic no turn geometry
                     [y + v * sin(heading)],   # basic no turn geometry
                     [        heading          ],   # no turning so heading = heading
                     [          v            ],   # velocity is constant
                     [     0.0000001         ]])  # Avoid divide by zero in JF
    else: # Take d_heading into account with nonlinear FX
        X = matrix([[x + v/turning * ( sin(heading + turning) - sin(heading))],
                    [ y + v/turning * (-cos(heading + turning) + cos(heading))],
                    [                   heading + turning                    ],
                    [                           v                             ],
                    [                       turning                           ]])
    x, y, heading, v, turning = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0]
    JF = eye(5)
    JF[0,2] =   v/turning * (cos(heading + turning) - cos(heading))
    JF[0,3] =  1./turning * (sin(heading + turning) - sin(heading))
    JF[0,4] = - v/(turning**2) * (sin(heading + turning) - sin(heading)) + v/turning * cos(heading + turning)
    JF[1,2] =   v/turning * (sin(heading + turning) - sin(heading))
    JF[1,3] =  1./turning * (-cos(heading + turning) + cos(heading))
    JF[1,4] = - v/(turning**2) * (-cos(heading + turning) + cos(heading)) + v/turning * sin(heading + turning)
    JF[2,4] = dt
    Q = diag([x_var**2, y_var**2, heading_var**2, v_var**2, turning_var**2])
    P = JF * P * JF.T + Q
    estimate_xy = [X[0,0], X[1,0]]
    return estimate_xy, X, P


def EKF_Update(measurement=[0.,0.], X=None, P=None, noise_est=0):
    if noise_est: xy_noise_var = noise_est
    else: xy_noise_var = 20.
    if type(X) == type(None): # Initialize X statespace
        X = matrix([[0.],  # x
                    [0.],  # y
                    [0.],  # heading (x_dir is 0 deg, y_dir is 90 deg)
                    [0.],  # velocity
                    [0.]]) # d_heading (positive is counter clockwise)
    if type(P) == type(None): # Initialize Uncertainty Matrix - no correlation uncertainty
        P = diag([1000., 1000., 2.*pi, 100., 2.*pi])
    Z = matrix([[float(measurement[0])],  # Only measures x, y.  Add sensors
                [float(measurement[1])],
                [         0.          ],
                [         0.          ],
                [         0.          ]]) # for heading, etc
    x, y, heading, v, turning = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0]
    HF = matrix([[x],  # Only measures x, y.  Add sensors
                 [y],
                 [0.],
                 [0.],
                 [0.]])  # for heading, etc
    JH = matrix([[1., 0., 0., 0., 0.],  # x row
                 [0., 1., 0., 0., 0.],
                 [0., 0., 0., 0., 0.],
                 [0., 0., 0., 0., 0.],
                 [0., 0., 0., 0., 0.]]) # y row. add rows for more sensors
    R = matrix([[xy_noise_var**2, 0., 0., 0., 0.],
                [0., xy_noise_var**2, 0., 0., 0.],
                [0.,    0., 0.001,    0.,     0.],
                [0.,    0.,    0., 0.001,     0.],
                [0.,    0.,    0.,    0.,  0.001]])
    I = eye(5)
    S = JH * P * JH.T + R
    K = (P * JH.T) * linalg.inv(S)
    Y = Z - HF
    X = X + (K * Y)
    P = (I - (K * JH)) * P
    estimate_xy = [X[0,0], X[1,0]]
    return estimate_xy, X, P


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    noise_est = 4. # should be 2x-4x noise variance
    if not OTHER:
        last_est_xy = target_measurement
        X = None
        P = None
        OTHER = last_est_xy, X, P
    else:
        last_est_xy, X, P = OTHER
    est_target_xy, X, P = EKF_Update(target_measurement, X, P, 1., noise_est)
    next_est_target_xy, X, P = EKF_Predict(X, P, dt = 1.)
    hunter_to_xy = next_est_target_xy # works if target will be within range
    dist_to_target = distance_between(next_est_target_xy, hunter_position)
    X_next, P_next = X.copy(), P.copy()
    for D in range( int( dist_to_target / max_distance ) ):
        hunter_to_xy, X_next, _ = EKF_Predict(X_next, P_next, 1.)
    turning = angle_trunc(get_heading(hunter_position, hunter_to_xy) - hunter_heading)
    distance = min(dist_to_target, max_distance)
    OTHER = next_est_target_xy, X, P
    return turning, distance, OTHER


