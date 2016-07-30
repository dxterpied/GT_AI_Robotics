from filterpy.kalman.sigma_points import *
import turtle
from robot import *
from numpy import eye, zeros, dot, isscalar, outer
from scipy.linalg import inv, cholesky
from filterpy.kalman import unscented_transform
from filterpy.common import dot3

# attempts to solve RR4 using UKF from KalmanPy
# based on RR2_UKF.py ---------------------------------------

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
measurement_noise = .05 * target.distance
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

        Parameters
        ----------

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
def fx(x, dt, turning):
    previousHeading = x[2]
    heading = previousHeading + turning
    x1 = x[0] + dt * cos(heading)
    y1 = x[1] + dt * sin(heading)
    state = [x1, y1, heading]
    return state


def Hx(x):
    result = x[0], x[1] , x[2] # x, y, heading
    return result


def residual_h(a, b):
    y = a - b
    for i in range(0, len(y), 3):
        y[i + 1] = normalize_angle(y[i + 1])
    return y


def residual_x(a, b):
    y = a - b
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

    sum_sin = np.sum( np.dot( np.sin(sigmas[:, 2] ), Wm) )
    sum_cos = np.sum( np.dot( np.cos(sigmas[:, 2] ), Wm) )
    x[2] = atan2(sum_sin, sum_cos)

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

    predictedPosition = [0, 0]
    xy_estimate = None

    if OTHER is None:
        distances = []
        angles = []
        coords = []
        xy_estimate = target_measurement
        steps = 0
        turnAngle = []
        ukf = None
    else:
        distances, angles, coords, xy_estimate, steps, turnAngle, ukf = OTHER

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
                points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)

                ukf = UKF(dim_x = 3, dim_z = 3, fx=fx, hx=Hx, points=points,
                          x_mean_fn=state_mean, z_mean_fn=z_mean,
                          residual_x= residual_x, residual_z= residual_h)

                ukf.x = np.array([target_measurement[0], target_measurement[1], headingAngle2])
                ukf.P = np.diag([.1, .1, .1])
                ukf.R = np.diag( [sigma_range**2, sigma_bearing**2, 3.] )
                ukf.Q = np.diag([.001, .001, .001])  # Q must not be zeroes!!! .001 is the best for this case


            ukf.predict(dt = avgDT, fx_args = rotationSign * avgAngle)
            z = [target_measurement[0], target_measurement[1], headingAngle2]
            ukf.update(z)

            newR = robot(ukf.x[0], ukf.x[1], ukf.x[2], rotationSign * avgAngle, avgDT)
            newR.move_in_circle()
            predictedPosition = newR.x, newR.y
            xy_estimate = newR.x, newR.y

            steps = 1

            while True:
                # check if hunter can catch target in straight line and calculate that point
                if (steps * max_distance) >= distance_between(hunter_position, xy_estimate) or steps > 50:
                    break
                steps += 1
                newR.move_in_circle()
                xy_estimate = newR.x, newR.y


    coords.append(target_measurement)
    OTHER = (distances, angles, coords, xy_estimate, steps, turnAngle, ukf)
    if xy_estimate is None:
        xy_estimate = target_measurement

    distance2 = distance_between(hunter_position, predictedPosition)

    if distance2 <= max_distance:
        turning = angle_trunc(get_heading(hunter_position, predictedPosition) - hunter_heading)
        distance = distance2
        OTHER = (distances, angles, coords, None, steps, turnAngle, ukf)
    else:
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
#     target = robot(0.0, 0.0, 0.0, -2*pi / 30, 1.5)
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

# This is better than Running Averages, PF, EKF !!!
# Clockwise:
# average score:  66.256
# minimum score:  12
# maximum score:  649
# fails:  0

# Counterclockwise:
# average score:  70.461
# minimum score:  21
# maximum score:  345
# fails:  0


#turtle.getscreen()._root.mainloop()