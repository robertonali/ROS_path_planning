#! /usr/bin/env python

import math
import matplotlib.pyplot as plt
import numpy as np
import bisect
import copy

# SPLINE
class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B

class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

    def calc_spline_course(x, y, ds=0.1):
        sp = Spline2D(x, y)
        s = list(np.arange(0, sp.s[-1], ds))

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            rk.append(sp.calc_curvature(i_s))

        return rx, ry, ryaw, rk, s

# QUINTIC
class QuinticPolynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # calc coefficient of quintic polynomial
        # See jupyter notebook document for derivation of this equation.
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt

def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
    """
    quintic polynomial planner

    input
        s_x: start x position [m]
        s_y: start y position [m]
        s_yaw: start yaw angle [rad]
        sa: start accel [m/ss]
        gx: goal x position [m]
        gy: goal y position [m]
        gyaw: goal yaw angle [rad]
        ga: goal accel [m/ss]
        max_accel: maximum accel [m/ss]
        max_jerk: maximum jerk [m/sss]
        dt: time tick [s]

    return
        time: time result
        rx: x position result list
        ry: y position result list
        ryaw: yaw angle result list
        rv: velocity result list
        ra: accel result list

    """

    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
            print("find path!!")
            break
    #este se vuela
    if show_animation:  # pragma: no cover
        for i, _ in enumerate(time):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.grid(True)
            plt.axis("equal")
            plot_arrow(sx, sy, syaw)
            plot_arrow(gx, gy, gyaw)
            plot_arrow(rx[i], ry[i], ryaw[i])
            plt.title("Time[s]:" + str(time[i])[0:4] +
                      " v[m/s]:" + str(rv[i])[0:4] +
                      " a[m/ss]:" + str(ra[i])[0:4] +
                      " jerk[m/sss]:" + str(rj[i])[0:4],
                      )
            plt.pause(0.001)

    return time, rx, ry, ryaw, rv, ra, rj

#los vamos a volar
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

# QUARTIC 
class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt

# FRENET
SIM_LOOP = 500

# Parameter
MAX_SPEED = 5.0  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 2.0  # maximum curvature [1/m] 
# MAX_ROAD_WIDTH = 7.0  # maximum road width [m] , calculated in LaserScan
D_ROAD_W = 0.4  # road width sampling length [m]
DT = 0.5  # time tick [s]
MAX_T = 3.0  # max prediction time [m]
MIN_T = 1.0  # min prediction time [m]
TARGET_SPEED = 3.0  # target speed [m/s]
D_T_S = 0.5  # target speed sampling length [m/s]
N_S_SAMPLE = 2  # sampling number of target speed
ROBOT_RADIUS = 0.4  # robot radius [m]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

class Frenet(object):
    """
    docstring
    """
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

# ROS - AQUI EMPIEZA NUESTRA CHAMBA, HACIA ABAJO
import rospy
import pandas as pd
import cv2
import time
import getpass
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from collections import defaultdict
from tf import transformations


class PID(object):
    def __init__(self, kp, ki, kd):
        self.dt       = 0.01
        self.setpoint = 0
        self.gains    = {'Kp': kp, 'Ki': ki, 'Kd': kd}
        self.error    = [0.0, 0.0]
        self.U        = 0.0
    
    def calculateControl(self, error):
        # self.setpoint = setpoint 
        self.error[0] = error
        Up = self.gains['Kp'] * self.error[0]
        Ui = self.gains['Ki'] * ((self.error[0] + self.error[1]) / 2) * self.dt
        Ud = self.gains['Kd'] * (self.error[0] - self.error[1]) * (1 / self.dt)

        self.U = Up + Ui + Ud

        self.error[1] = self.error[0]
        # return min(max(-max_val, self.U), max_val)

class Orientation(object):
        def __init__(self):
            self.quaternion = list()
            self.euler      = defaultdict(lambda: float)
            
class Odom(object):
    def __init__(self):
        self.waypoints   = np.genfromtxt('./ros_wall_follower/scripts/csv/odom_data_frenet.csv', delimiter=',')
        self.current_pos = {'x': 0.0, 'y': 0.0, 'x2': 0.0, 'y2': 0.0}
        self.prev_pos    = {'x': 0.0, 'y': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'total': 0.0}
        self.orientation = Orientation() 
        self.track       = 0.28
        self.wheelbase   = 0.40
        
class Steering(object):
    def __init__(self):
        self.max_steering    = 1.22
        self.steering_output = 0.0

class FrenetPurePursuit(PID, Odom, Steering):
    def __init__(self):
        PID.__init__(self, 2.15, 0.00, 0.75)
        Odom.__init__(self)
        Steering.__init__(self)

        rospy.init_node("dummy_agent")
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)
        rospy.Timer(rospy.Duration(0.1), self.timerPurePursuitCallback)

        self.acker_msg       = AckermannDriveStamped()
        # LateralKinematics variables
        self.ld     = 0.0
        self.ldcalc = 0.0
        self.kdd    = 0.30   # 0.40
        self.alpha  = 0.0    # Angle between actual car position and waypoint ahead (gama - yaw)
        self.gamma  = 0.0    # Angle between 0deg and waypoint ahead
        self.delta  = 0.0    # Turning wheels angle.
        self.crosstrack_error = 0.0
        self.orientation.euler['yaw'] = 0.0

        # LaserScan
        self.MAX_ROAD_WIDTH = 5.0
        self.laser_angle_range = 0
        self.laser_step = 25
        self.laser_obs = list() # Diccionario de lista x, y 
        self.obstacles_ready = False
        self.obstacles = 0 # np array

        # Frenet
        self.wx = 0
        self.wy = 0
            # Initial State
        self.c_speed = 5.0 # current speed [m/s]
        self.c_d = 0.0     # current lateral position [m]
        self.c_d_d = 0.0   # current lateral speed [m/s]
        self.c_d_dd = 0.0  # current lateral acceleration [m/s]
        self.s0 = 0.0      # current course position

    
    def odomCallback(self, msg):
        self.current_pos['x'] = msg.pose.pose.position.x
        self.current_pos['y'] = msg.pose.pose.position.y
        self.current_vel['x'] = msg.twist.twist.linear.x
        self.current_vel['y'] = msg.twist.twist.linear.y
        self.current_vel['total'] = np.hypot(self.current_vel['x'], self.current_vel['y'])
        
        self.orientation.quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (self.orientation.euler['roll'], self.orientation.euler['roll'],
            self.orientation.euler['yaw']) = transformations.euler_from_quaternion(self.orientation.quaternion)

        self.current_pos['x2'] = self.current_pos['x'] - ((self.wheelbase / 2) * np.cos(self.orientation.euler['yaw']))
        self.current_pos['y2'] = self.current_pos['y'] - ((self.wheelbase / 2) * np.sin(self.orientation.euler['yaw']))

    def laserCallback(self, msg):
        if(self.obstacles_ready == False):
            # 0 - 1079
            self.laser_angle_range = np.linspace(0, 1079, self.laser_step) #216
            # Sacar distancias 
            for i in self.laser_angle_range:
                distance = msg.ranges[int(i)]
                if (distance <= 15.0): # modificar distancia horizonte
                    phi = (i*0.25) - 135.0
                    x = distance * np.cos(phi)
                    y = -1.0*distance * np.sin(phi)
                    # Global frame
                    x = x + self.current_pos['x']
                    y = y + self.current_pos['y']

                    # self.laser_obs['x'].append(x)
                    # self.laser_obs['y'].append(y)
                    self.laser_obs.append([x , y])
                    self.obstacles_ready = True

            self.MAX_ROAD_WIDTH = msg.ranges[180] + msg.ranges[900]


    def timerCallback(self, event):
        # if self.process_ready:
        #     if not self.move:
        #         self.move = True
        #         self.index = 0
        #         self.process_ready = False
        # if self.finish_wp and self.move:
        #     self.finish_wp = False
        #     self.temp_wp = self.wp_pp
        #     self.size_temp_pp =  self.size_wp_pp
        # if self.move:
        #     self.takeAction()
        #     self.drive_pub.publish(self.acker_msg)
        pass

# Frenet    
    def calc_frenet_paths(self, c_speed, c_d, c_d_d, c_d_dd, s0):
        frenet_paths = []

        # generate path to each offset goal
        for di in np.arange(-self.MAX_ROAD_WIDTH/2, self.MAX_ROAD_WIDTH/2, D_ROAD_W):

            # Lateral motion planning
            for Ti in np.arange(MIN_T, MAX_T, DT): # Horizonte lateral
                fp = Frenet()

                # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
                lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.t = [t for t in np.arange(0.0, Ti, DT)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Longitudinal motion planning (Velocity keeping)
                for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                    TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                    tfp = copy.deepcopy(fp)
                    lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # square of diff from target speed
                    ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                    tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                    tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                    tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                    frenet_paths.append(tfp)

        return frenet_paths


    def calc_global_paths(self, fplist, csp):
        for fp in fplist:

            # calc global positions
            for i in range(len(fp.s)):
                ix, iy = csp.calc_position(fp.s[i])
                if ix is None:
                    break
                i_yaw = csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)

            # calc yaw and ds
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

        return fplist


    def check_collision(self, fp, ob):
        ob_nuevo = np.asarray(self.laser_obs)
        for i in range(len(ob_nuevo[:, 0])):
            d = [((ix - ob_nuevo[i, 0]) ** 2 + (iy - ob_nuevo[i, 1]) ** 2)
                for (ix, iy) in zip(fp.x, fp.y)]

            collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

            if collision:
                return False

        return True

    def check_paths(self, fplist, ob):
        ok_ind = []
        for i, _ in enumerate(fplist):
            if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
                continue
            elif any([abs(a) > MAX_ACCEL for a in
                    fplist[i].s_dd]):  # Max accel check
                continue
            elif any([abs(c) > MAX_CURVATURE for c in
                    fplist[i].c]):  # Max curvature check
                continue
            elif not self.check_collision(fplist[i], ob):
                continue

            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]


    def frenet_optimal_planning(self, csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
        import pdb; pdb.set_trace()
        fplist = self.calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
        import pdb; pdb.set_trace()
        fplist = self.calc_global_paths(fplist, csp)
        import pdb; pdb.set_trace()
        fplist = self.check_paths(fplist, ob)
        import pdb; pdb.set_trace()

        # fpoints = list()
        # for i in fplist:
        #     for j in range(len(i.x)): 
        #         fpoints.append([i.x[j], i.y[j], i.s_d[j]])
        # np.savetxt('./ros_wall_follower/scripts/csv/frenet_path_8.csv', fpoints, delimiter = ",")
        # import pdb; pdb.set_trace()

        # find minimum cost path
        min_cost = float("inf")
        best_path = None
        for fp in fplist:
            if min_cost >= fp.cf:
                min_cost = fp.cf
                best_path = fp
        
        # fpoints = list()
        # for j in range(len(best_path.x)): 
        #     fpoints.append([best_path.x[j], best_path.y[j], best_path.s_d[j], best_path.d_d[j]])
        # np.savetxt('./ros_wall_follower/scripts/csv/frenet_bestpath_5.csv', fpoints, delimiter = ",")
        # import pdb; pdb.set_trace()

        return best_path # Entrada pure pursuit
        

    def generate_target_course(self, x, y):
        csp = Spline2D(x, y)
        s = np.arange(0, csp.s[-1], 0.1)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(csp.calc_yaw(i_s))
            rk.append(csp.calc_curvature(i_s))

        return rx, ry, ryaw, rk, csp

    def main(self):
        # waypoints ready
        self.wx = self.waypoints[:, 0]
        self.wy = self.waypoints[:, 1]
        # obstacles ready
        # obstacles = np.asarray(self.laser_obs)
        # Generate spline course

        tx, ty, tyaw, tc, csp = self.generate_target_course(self.wx, self.wy) 

        path = self.frenet_optimal_planning(
            csp, self.s0, self.c_speed, self.c_d, self.c_d_d, self.c_d_dd, np.asarray(self.laser_obs))

        self.s0 = path.s[1]
        self.c_d = path.d[1]
        self.c_d_d = path.d_d[1]
        self.c_d_dd = path.d_dd[1]
        self.c_speed = path.s_d[1]

# Pure Pursuit
    def timerPurePursuitCallback(self, event):
        # self.contProcess()
        pass

    def takeAction(self):
        if (np.hypot((self.temp_wp[-1, 0] - self.current_pos['x2']),
                            (self.temp_wp[-1, 1] - self.current_pos['y2']))
                                <= 3.0): # Distancia Umbral
            if not self.as_active and not self.process_ready:
                rospy.loginfo("UMBRAL ACTIVADO")
                self.as_active = True
        else:
            pass

        # if (math.sqrt((self.waypoints[self.wp_index, 0] - self.x2) ** 2 + ((self.waypoints[self.wp_index, 1] - self.y2) ** 2))
        #         <= 1.2) :
        if (np.hypot((self.temp_wp[self.index, 0] - self.current_pos['x2']),
                            (self.temp_wp[self.index, 1] - self.current_pos['y2']))
                                <= 1.2):
            # Go to next target by adding one to the index of the waypoints list
            if self.index < self.size_temp_pp - 1:
                self.index += 1
                rospy.loginfo("SIZES")
                rospy.loginfo(self.index)
                rospy.loginfo(self.size_temp_pp)
            else:
                rospy.loginfo(self.index)
                rospy.loginfo(self.size_temp_pp)
                rospy.loginfo("Detente")
                self.move = False
                self.finish_wp = True
                self.index = 0
        else:
            pass

        self.vel = 2.0 if self.move else 0.0             # self.vel = 1.0 #- abs(self.steering_output) * 3.0/1.22

        if self.move:
            self.calcLateralKinematics()
            self.calculateControl(self.crosstrack_error)
            self.delta = np.arctan2( (2 * self.wheelbase * (self.U)), (self.ld)**2 )
            self.steering_output = self.delta # min(max(-self.max_steering, self.delta), self.max_steering)
            self.setCarMovement(self.steering_output, 0.00, self.vel, 0.0, 0.0)
        
        # rospy.loginfo("LDCacl:{} , LD: {}, VEL: {} ".format(self.ldcalc, self.ld, self.current_vel['total']))
        # rospy.loginfo("Steer: {}, Delta: {}".format(self.steering_output, self.delta))

    def calcLateralKinematics(self):
        # self.ld = math.sqrt((self.waypoints[self.wp_index,0] - self.current_pos['x2']) ** 2 + ((self.waypoints[self.wp_index,1] - self.y2) ** 2))
        self.ld     = np.hypot((self.temp_wp[self.index, 0] - self.current_pos['x2']), (self.temp_wp[self.index, 1] - self.current_pos['y2']))
        self.gama   = math.atan2((self.temp_wp[self.index, 1] - self.current_pos['y2']), (self.temp_wp[self.index, 0] - self.current_pos['x2']))
        self.alpha  = self.gama - self.orientation.euler['yaw']
        self.ldcalc = (self.kdd) * (self.current_vel['total']) 
        self.delta  = math.atan2((2 * self.wheelbase * math.sin(self.alpha)), (self.ldcalc))
        self.crosstrack_error = self.ldcalc * math.sin(self.alpha)
    
    def setCarMovement(self, steering_angle, steering_angle_velocity, speed,
                        acceleration, jerk):
        self.acker_msg.drive.steering_angle = steering_angle
        # self.acker_msg.drive.steering_angle_velocity= steering_angle_velocity
        self.acker_msg.drive.speed = speed
        # self.acker_msg.drive.acceleration = acceleration
        # self.acker_msg.drive.jerk = jerk

    def contProcess(self):
        if self.first_loop:
            rospy.loginfo("Init")
            self.first_loop = False
            self.get_points()
            self.ready = True
        if (self.as_active and self.ready and self.num < (self.index2)):
            rospy.loginfo("Process")
            s = time.time() 
            self.process()
            e = time.time()
            print(e-s)
            # self.index = 0
            # self.move = True
        rospy.loginfo("Paso")

if __name__ == "__main__":
    robot = FrenetPurePursuit()
    robot.main()
    rospy.spin()