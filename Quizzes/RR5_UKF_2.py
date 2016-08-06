
import matplotlib
matplotlib.use('TkAgg')

from filterpy.kalman.sigma_points import *
import turtle
from robot import *
from scipy.linalg import inv, cholesky
from filterpy.kalman import unscented_transform
from filterpy.common import dot3
from numpy import *

# attempts to solve RR5 using UKF from KalmanPy
# based on RR4_UKF.py ---------------------------------------

# Fails miserably...........

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
measured_robot = turtle.Turtle()
measured_robot.shape('circle')
measured_robot.color('red')
measured_robot.shapesize(0.2, 0.2, 0.2)

target = robot(0.0, 15.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2.0 * target.distance
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -5.0, 0.0)

sigma_vel=0.1
sigma_steer= np.radians(1)
sigma_range= 0.3
sigma_bearing= 0.1

class UKF(object):
    """
    Attributes
    ----------
    x : numpy.array(dim_x)
        state estimate vector
    P : numpy.array(dim_x, dim_x)
        covariance estimate matrix
    R : numpy.array(dim_z, dim_z)
        measurement noise matrix
    Q : numpy.array(dim_x, dim_x)
        process noise matrix

    Readable Attributes
    -------------------
    xp : numpy.array(dim_x)
        predicted state (result of predict())
    Pp : numpy.array(dim_x, dim_x)
        predicted covariance matrix (result of predict())
    """

    def __init__(self, dim_x, dim_z, hx, fx, points, x_mean_fn=None, z_mean_fn=None,
                 residual_x=None,
                 residual_z=None):
        r"""
        Parameters
        ----------

        dim_x : int
            Number of state variables for the filter. For example, if
            you are tracking the position and velocity of an object in two
            dimensions, dim_x would be 4.
        dim_z : int
            Number of of measurement inputs. For example, if the sensor
            provides you with position in (x,y), dim_z would be 2.
        dt : float
            Time between steps in seconds.
        hx : function(x)
            Measurement function. Converts state vector x into a measurement
            vector of shape (dim_z).
        fx : function(x,dt)
            function that returns the state x transformed by the
            state transistion function. dt is the time step in seconds.
        points : class
            Class which computes the sigma points and weights for a UKF
            algorithm. You can vary the UKF implementation by changing this
            class. For example, MerweScaledSigmaPoints implements the alpha,
            beta, kappa parameterization of Van der Merwe, and
            JulierSigmaPoints implements Julier's original kappa
            parameterization. See either of those for the required
            signature of this class if you want to implement your own.
        x_mean_fn : callable  (sigma_points, weights), optional
            Function that computes the mean of the provided sigma points
            and weights. Use this if your state variable contains nonlinear
            values such as angles which cannot be summed.

            .. code-block:: Python
                def state_mean(sigmas, Wm):
                    x = np.zeros(3)
                    sum_sin, sum_cos = 0., 0.

                    for i in range(len(sigmas)):
                        s = sigmas[i]
                        x[0] += s[0] * Wm[i]
                        x[1] += s[1] * Wm[i]
                        sum_sin += sin(s[2])*Wm[i]
                        sum_cos += cos(s[2])*Wm[i]
                    x[2] = atan2(sum_sin, sum_cos)
                    return x

        z_mean_fn : callable  (sigma_points, weights), optional
            Same as x_mean_fn, except it is called for sigma points which
            form the measurements after being passed through hx().

        residual_x : callable (x, y), optional
        residual_z : callable (x, y), optional
            Function that computes the residual (difference) between x and y.
            You will have to supply this if your state variable cannot support
            subtraction, such as angles (359-1 degreees is 2, not 358). x and y
            are state vectors, not scalars. One is for the state variable,
            the other is for the measurement state.

            .. code-block:: Python

                def residual(a, b):
                    y = a[0] - b[0]
                    if y > np.pi:
                        y -= 2*np.pi
                    if y < -np.pi:
                        y = 2*np.pi
                    return y

        """

        self.Q = eye(dim_x)
        self.R = eye(dim_z)
        self.x = zeros(dim_x)
        self.P = eye(dim_x)
        self._dim_x = dim_x
        self._dim_z = dim_z
        self._num_sigmas = 2*dim_x + 1
        self.hx = hx
        self.fx = fx
        self.points_fn = points
        self.x_mean = x_mean_fn
        self.z_mean = z_mean_fn
        self.msqrt = cholesky

        # weights for the means and covariances.
        self.Wm, self.Wc = self.points_fn.weights()

        if residual_x is None:
            self.residual_x = np.subtract
        else:
            self.residual_x = residual_x

        if residual_z is None:
            self.residual_z = np.subtract
        else:
            self.residual_z = residual_z

        # sigma points transformed through f(x) and h(x)
        # variables for efficiency so we don't recreate every update
        self.sigmas_f = zeros((2*self._dim_x+1, self._dim_x))
        self.sigmas_h = zeros((self._num_sigmas, self._dim_z))



    def predict(self, dt,  fx_args=()):
        r""" Performs the predict step of the UKF. On return, self.x and
        self.P contain the predicted state (x) and covariance (P). '

        Important: this MUST be called before update() is called for the first
        time.

        dt : double, optional
            If specified, the time step to be used for this prediction.
            self._dt is used if this is not provided.

        UT : function(sigmas, Wm, Wc, noise_cov), optional
            Optional function to compute the unscented transform for the sigma
            points passed through hx. Typically the default function will
            work - you can use x_mean_fn and z_mean_fn to alter the behavior
            of the unscented transform.

        fx_args : tuple, optional, default (,)
            optional arguments to be passed into fx() after the required state
            variable.
        """

        if not isinstance(fx_args, tuple):
            fx_args = (fx_args,)

        # calculate sigma points for given mean and covariance
        sigmas = self.points_fn.sigma_points(self.x, self.P)

        for i in range(self._num_sigmas):
            self.sigmas_f[i] = self.fx(sigmas[i], dt, *fx_args)

        self.x, self.P = unscented_transform(self.sigmas_f, self.Wm, self.Wc, self.Q,
                            self.x_mean, self.residual_x)


    def update(self, z, R=None, hx_args=()):
        """ Update the UKF with the given measurements. On return,
        self.x and self.P contain the new mean and covariance of the filter.

        Parameters
        ----------

        z : numpy.array of shape (dim_z)
            measurement vector

        R : numpy.array((dim_z, dim_z)), optional
            Measurement noise. If provided, overrides self.R for
            this function call.

        UT : function(sigmas, Wm, Wc, noise_cov), optional
            Optional function to compute the unscented transform for the sigma
            points passed through hx. Typically the default function will
            work - you can use x_mean_fn and z_mean_fn to alter the behavior
            of the unscented transform.

        hx_args : tuple, optional, default (,)
            arguments to be passed into Hx function after the required state
            variable.
        """

        if not isinstance(hx_args, tuple):
            hx_args = (hx_args,)

        if R is None:
            R = self.R
        elif isscalar(R):
            R = eye(self._dim_z) * R

        for i in range(self._num_sigmas):
            self.sigmas_h[i] = self.hx(self.sigmas_f[i], *hx_args)

        # mean and covariance of prediction passed through unscented transform
        zp, Pz = unscented_transform(self.sigmas_h, self.Wm, self.Wc, R, self.z_mean, self.residual_z)

        # compute cross variance of the state and the measurements
        Pxz = zeros((self._dim_x, self._dim_z))
        for i in range(self._num_sigmas):
            dx = self.residual_x(self.sigmas_f[i], self.x)
            dz =  self.residual_z(self.sigmas_h[i], zp)
            Pxz += self.Wc[i] * outer(dx, dz)

        K = dot(Pxz, inv(Pz))   # Kalman gain
        y = self.residual_z(z, zp)   #residual
        self.x = self.x + dot(K, y)
        self.P = self.P - dot3(K, Pz, K.T)


def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x

# state transition function
def fx_1(X, dt):
    x = X[0]
    y = X[1]
    heading = X[2]
    velocity = X[3]
    turning = X[4]

    heading = heading + turning
    x =  x + dt * cos(heading)
    y =  y + dt * sin(heading)

    state = [x, y, heading, velocity, turning]

    print state
    return state




def fx(X, dt):
    x = X[0]
    y = X[1]
    heading = X[2]
    velocity = X[3]
    turning = X[4]

    if abs(turning) < 0.0001:
        x =  x +  dt * cos(heading)   #x[0] + dt * cos(heading)
        y =  y +  dt * sin(heading)
    else:
        # x =  x + velocity/turning * ( sin(heading + turning*dt) - sin(heading))   #x[0] + dt * cos(heading)
        # y =  y + velocity/turning * (-cos(heading + turning*dt) + cos(heading))
        # heading = heading + turning * dt
        heading = heading + turning
        x =  x +  dt * cos(heading)   #x[0] + dt * cos(heading)
        y =  y +  dt * sin(heading)

    state = [x, y, heading, velocity, turning]
    print state
    return state


def Hx(x):
    result = x[0], x[1], x[2], x[3], x[4]
    return result


def residual_h(a, b):
    y = a - b
    for i in range(0, len(y), 3):
        y[i + 1] = normalize_angle(y[i + 1])
    return y


def residual_x(z, zp):
    y = z - zp
    y[2] = normalize_angle(y[2])
    return y


# sigmas here has three columns - for x, y, and heading
def state_mean(sigmas, Wm):
    x = np.zeros(sigmas.shape[1])

    x[0] = np.sum( np.dot(sigmas[:, 0], Wm) )
    x[1] = np.sum( np.dot(sigmas[:, 1], Wm) )

    sum_sin = np.sum( np.dot( np.sin(sigmas[:, 2] ), Wm) )
    sum_cos = np.sum( np.dot( np.cos(sigmas[:, 2] ), Wm) )
    x[2] = atan2(sum_sin, sum_cos)

    return x


# sigmas here has two columns - one for x and one for y
def z_mean(sigmas, Wm):
    x = np.zeros(sigmas.shape[1])

    # for z in range(0, z_count, 2):
    #     sum_sin = np.sum(np.dot(np.sin(sigmas[:, z+1]), Wm))
    #     sum_cos = np.sum(np.dot(np.cos(sigmas[:, z+1]), Wm))
    #
    #     x[z] = np.sum(np.dot(sigmas[:,z], Wm))
    #     x[z+1] = atan2(sum_sin, sum_cos)

    x[0] = np.sum( np.dot(sigmas[:, 0], Wm) )
    x[1] = np.sum( np.dot(sigmas[:, 1], Wm) )


    return x


# cross product
def calculateRotationDirection(Ax, Ay, Bx, By, Cx, Cy):
    return ((Bx - Ax) * (Cy - By)) - ((By - Ay) * (Cx - Bx))


def getRotationSign(rotationAngles):
    # some will be negative; some positive; count which one has more elements
    positive = [i for i in rotationAngles if i > 0.0]
    negative = [i for i in rotationAngles if i < 0.0]

    if len(positive) > len(negative):
        return 1
    else:
        return -1



def next_move_straight_line(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):

    xy_estimate = target_measurement

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        turnAngle = []
        ukf = None
    else:
        distances, angles, coords, xy_estimate, steps, turnAngle, ukf= OTHER

        if len(coords) == 1:
            hypotenuse1 = distance_between(coords[0], target_measurement)
            distances.append(hypotenuse1)
            xy_estimate = target_measurement

        elif len(coords) >= 2:
            point1 = coords[len(coords) - 2]
            point2 = coords[len(coords) - 1]
            point3 = target_measurement

            # this is done to determine clock wise or counter clock wise rotation
            turnAngle.append(calculateRotationDirection(point1[0], point1[1], point2[0], point2[1], point3[0], point3[1]))
            rotationSign = getRotationSign(turnAngle)

            y1Delta = point2[1] - point1[1]
            hypotenuse1 = distance_between(point1, point2)
            headingAngleAvg1 = asin(y1Delta / hypotenuse1)

            y2Delta = point3[1] - point2[1]
            x2Delta = point3[0] - point2[0]
            hypotenuse2 = distance_between(point2, point3)
            headingAngle2 = atan2(y2Delta, x2Delta)
            headingAngleAvg2 = asin(y2Delta / hypotenuse2)
            predictedTurnAngleAvg = headingAngleAvg2 - headingAngleAvg1

            angles.append(abs(predictedTurnAngleAvg))
            distances.append(hypotenuse2)

            avgDT = sum(distances)/len(distances)
            avgAngle = sum(angles)/len(angles)

            if ukf is None:
                points = MerweScaledSigmaPoints(n=5, alpha=.00001, beta=2, kappa=0, subtract=residual_x)

                ukf = UKF(dim_x = 5, dim_z = 5, fx=fx, hx=Hx, points=points,
                          x_mean_fn=state_mean, z_mean_fn=z_mean,
                          residual_x= residual_x, residual_z= residual_h)

                ukf.x = np.array([target_measurement[0], target_measurement[1], 0., 0., 0.]) # x, y, heading, velocity, turning
                ukf.P = np.diag([1., 1., 1., 1., 1.])
                ukf.R = np.diag( [sigma_range**2, sigma_bearing**2, .0001, 0.0001, .0001] )
                ukf.Q = np.diag([.001, .001, .001, .001, .001])  # Q must not be zeroes!!! .001 is the best for this case


            ukf.predict(dt = 1.)
            z = [target_measurement[0], target_measurement[1], 0., 0., 0.]
            ukf.update(z)

            # newR = robot(ukf.x[0], ukf.x[1], ukf.x[2], ukf.x[4], ukf.x[3])
            # newR.move_in_circle()
            xy_estimate = ukf.x[0], ukf.x[1]

            steps = 1
            while True:
                # check how many steps it will take to get there for Hunter
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1
                #newR.move_in_circle()
                ukf.predict(dt = 1.)
                xy_estimate = ukf.x[0], ukf.x[1]


    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps, turnAngle, ukf)
    if xy_estimate is None:
        xy_estimate = target_measurement

    turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading)
    distance = distance_between(hunter_position, xy_estimate)


    return turning, distance, OTHER



# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):

    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)

        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            return ctr
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
            return 1000
    return caught


def demo_grading_visual(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization

    turtle.setup(800, 800)
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 20.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.2, 0.2, 0.2)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.2, 0.2, 0.2)
    #prediction.penup()
    broken_robot.penup()
    #End of Visualization

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # if ctr >= 43:
        #     #time.sleep(0.5)
        #     prediction.color('red')
        #     broken_robot.color('black')

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        #print "actual target position", target_position
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        #print ctr + 1

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 200)
        prediction.stamp()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."

    return caught


#demo_grading(hunter, target, next_move_straight_line)
demo_grading_visual(hunter, target, next_move_straight_line)


# scores = []
# fails = 0
# for i in range(1000):
#     print i
#     target = robot(0.0, 0.0, 0.0, 2*pi / 30, 1.5)
#     target.set_noise(0.0, 0.0, measurement_noise)
#     hunter = robot(-10.0, -20.0, 0.0)
#     score = demo_grading(hunter, target, next_move_straight_line)
#     if score == 1000:
#         fails += 1
#     else:
#         scores.append(score)
#
# print "average score: ", sum(scores)/ float(len(scores))
# print "minimum score: ", min(scores)
# print "maximum score: ", max(scores)
# print "fails: ", fails


# x = [1.3905235974633778, 7.832049402391674, 9.020314462351989, 3.687485010010496, 4.933087914504979, 5.842278721831094, 7.374384722598193, 11.548435059429593, 10.08330407331331, 4.048332216167037, 11.753400231528877, 5.24814930818585, 0.3780861370621249, 3.819226471192638, 3.219009627591963, -0.660991670650813, -6.810902143188039, -2.648632039619782, -0.8651469995661083, -5.882826870804094, -6.771881598941817, -6.883212384180803, -10.970288387596014, -9.38416048883138, -2.0041408634734, -4.1999609273146294, -4.427392485133488, -4.094107923812431, -4.539507495972897, -6.958816977357863, -0.5837156626237164, 3.211904185873981, 2.6999090897786973, 4.946462347786874, 6.250544373196611, 1.4360007457367132, 6.376447944684558, 5.395872942944427, 3.6592834591198713, 3.6799054753035985, 6.372070482657179, 10.83971503990248, 6.471684767984395, -4.13122094642976, 1.3657905443071188, -2.1545837703277186, -3.6218498314375362, -5.990316163817381, -2.825131401522447, -8.47962049553197, -3.0710249464300077, -5.242374500395425, -11.356227320748491, -3.580215175474069, -8.949524398862625, -12.148475712671257, -3.541357682569754, -2.3218551317498113, -4.10750098248105, -1.154362522019348, 1.719708807469195, -2.9317706605909493, 5.581580285882415, 7.322885401113478, 8.035547065332059, 7.115626723235949, 6.8834109751060195, 9.605681429819468, 5.378480946301948, 4.55936497171603, 0.9696632799355314, 3.0281397205843197, -2.4315041509242974, -0.4213417059299991, 0.13371487118430309, -2.5616840637149783, -3.330922819747191, -3.238792470549532, -12.848697164298123, -11.650132469341331, -2.9250345334239203, -8.844969766009035, -7.54940178448948, -10.204223020208033, -7.4851192397126525, -9.156624882388012, -6.366425519643718, -8.553734125327592, -4.609302566545153, -4.264881667574036, 1.4533519725675805]
# y = [5.1592964225372135, 8.216934337621904, 14.221677454416298, 9.606966741201592, 15.393598119630198, 15.463991661062682, 18.798461424361072, 17.83425651827887, 16.480645145332982, 22.97048479933387, 19.510051232214657, 24.642699320629887, 25.45829257417096, 25.497397786842278, 22.450305133766594, 21.947846746481055, 18.037770599424036, 23.858011123610375, 19.308656588152285, 16.69873918296076, 20.488408553230123, 16.367880867164754, 23.654044378766407, 20.73648776720881, 15.90450795825495, 16.135338583542634, 8.493714748632618, 8.913938224865586, 8.734285273935438, 8.050993570472942, 10.158164230122795, 8.08449276240243, 14.886302280598855, 14.237310205824675, 17.306486687897937, 13.8601024213346, 17.90588766410904, 15.869047349619871, 20.686353443244506, 18.42398071352659, 23.22554467544738, 21.198830063607687, 24.727001975565397, 23.188652263477202, 25.512192853204706, 25.07747201310473, 24.054712728944807, 22.68899006890544, 22.641944066718526, 18.33228864387304, 17.94999176153331, 22.10590333682616, 19.005801408713417, 13.582191596376182, 10.639031293990215, 15.577633926934181, 9.911438955202737, 8.666040822401431, 11.027340108357107, 12.56573800372876, 11.771468252890216, 12.46635569231553, 8.986038622183006, 12.215863938535502, 10.291879983793017, 16.505270167068254, 19.6009636347262, 19.10972604819267, 15.892910888479538, 26.63984263799601, 17.951774058138326, 21.107945349676598, 23.441244468726836, 21.853959925952115, 21.162499359087953, 21.808181391384416, 20.52011983367284, 19.153098462721978, 26.471740153772178, 25.208554508505728, 17.67356964440606, 18.51548339755294, 16.646498463883713, 15.599794378503121, 17.747778170685166, 13.998749493955843, 13.588001144109704, 16.94352162086974, 9.476077823803571, 5.4278110057897955, 12.342893333479008]

x = [1.3905235974633778, 7.832049402391674, 9.020314462351989, 3.687485010010496, 4.933087914504979, 5.842278721831094]
y = [5.1592964225372135, 8.216934337621904, 14.221677454416298, 9.606966741201592, 15.393598119630198, 15.463991661062682]

x_actual = [0.0, 1.4672214011007085, 2.83753958756461, 4.0510650791270315, 5.054760988665319, 5.80476098866532, 6.268286480227742, 6.4250791751292216, 6.268286480227741, 5.804760988665318, 5.054760988665317, 4.051065079127029, 2.837539587564608, 1.4672214011007072, -8.881784197001252e-16, -1.5000000000000009, -2.9672214011007103, -4.337539587564613, -5.551065079127038, -6.554760988665329, -7.304760988665335, -7.768286480227762, -7.925079175129248, -7.768286480227774, -7.3047609886653575, -6.55476098866536, -5.551065079127074, -4.3375395875646525, -2.9672214011007503, -1.500000000000041, -4.107825191113079e-14, 1.4672214011006668, 2.837539587564567, 4.051065079126986, 5.054760988665272, 5.8047609886652705, 6.268286480227689, 6.4250791751291665, 6.268286480227683, 5.804760988665258, 5.0547609886652545, 4.051065079126965, 2.8375395875645424, 1.4672214011006404, -6.816769371198461e-14, -1.5000000000000682, -2.967221401100777, -4.337539587564679, -5.551065079127102, -6.554760988665391, -7.304760988665395, -7.76828648022782, -7.925079175129303, -7.768286480227826, -7.304760988665407, -6.55476098866541, -5.551065079127124, -4.337539587564702, -2.9672214011008, -1.5000000000000908, -9.08162434143378e-14, 1.467221401100617, 2.837539587564517, 4.0510650791269365, 5.0547609886652225, 5.804760988665221, 6.26828648022764, 6.425079175129117, 6.2682864802276335, 5.804760988665208, 5.054760988665205, 4.051065079126915, 2.8375395875644926, 1.4672214011005906, -1.1790568521519162e-13, -1.500000000000118, -2.9672214011008267, -4.337539587564729, -5.551065079127151, -6.554760988665441, -7.3047609886654445, -7.76828648022787, -7.925079175129353, -7.768286480227876, -7.304760988665457, -6.55476098866546, -5.551065079127174, -4.337539587564752, -2.9672214011008498, -1.5000000000001406, -1.4055423491754482e-13]
y_actual = [10.0, 10.31186753622664, 10.92197250084034, 11.803650379279048, 12.91836761749514, 14.217405723171797, 15.643990497614528, 17.135773340666937, 18.627556183719346, 20.054140958162076, 21.353179063838734, 22.467896302054825, 23.349574180493534, 23.959679145107234, 24.271546681333874, 24.271546681333877, 23.95967914510724, 23.349574180493544, 22.46789630205484, 21.353179063838752, 20.054140958162098, 18.62755618371937, 17.13577334066696, 15.643990497614551, 14.217405723171819, 12.918367617495159, 11.803650379279066, 10.921972500840356, 10.311867536226657, 10.000000000000021, 10.000000000000025, 10.311867536226666, 10.921972500840369, 11.80365037927908, 12.918367617495173, 14.217405723171833, 15.643990497614563, 17.135773340666972, 18.62755618371938, 20.05414095816211, 21.353179063838766, 22.467896302054854, 23.349574180493562, 23.95967914510726, 24.2715466813339, 24.2715466813339, 23.95967914510726, 23.349574180493562, 22.467896302054854, 21.353179063838766, 20.05414095816211, 18.62755618371938, 17.135773340666972, 15.643990497614562, 14.217405723171831, 12.918367617495171, 11.803650379279079, 10.921972500840369, 10.31186753622667, 10.000000000000034, 10.000000000000037, 10.311867536226679, 10.921972500840381, 11.803650379279093, 12.918367617495186, 14.217405723171845, 15.643990497614576, 17.135773340666987, 18.627556183719395, 20.054140958162126, 21.35317906383878, 22.467896302054868, 23.349574180493576, 23.959679145107273, 24.271546681333913, 24.271546681333913, 23.959679145107273, 23.349574180493576, 22.467896302054868, 21.35317906383878, 20.054140958162126, 18.627556183719395, 17.135773340666987, 15.643990497614576, 14.217405723171845, 12.918367617495186, 11.803650379279093, 10.921972500840383, 10.311867536226684, 10.000000000000048, 10.000000000000052]


#least_squares(x, y, x_actual, y_actual)


#turtle.getscreen()._root.mainloop()