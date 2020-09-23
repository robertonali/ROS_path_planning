#! /usr/bin/env python

import pdb
import rospy
import math
import numpy as np
import cv2
import sys
import getpass
import time
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

class Steering(object):
    def __init__(self):
        self.max_steering    = 1.22
        self.steering_output = 0.0

class Orientation(object):
        def __init__(self):
            self.quaternion = list()
            self.euler      = defaultdict(lambda: float)

class Odom(object):
    def __init__(self):
        self.current_pos = {'x': 0.0, 'y': 0.0, 'x2': 0.0, 'y2': 0.0}
        self.prev_pos    = {'x': 0.0, 'y': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'total': 0.0}
        self.orientation = Orientation() 
        self.track       = 0.28
        self.wheelbase   = 0.40

class PotentialField(object):
    def __init__(self):
        self.end      = list()
        self.start    = [70, 232]
        self.index2   = 0
        self.rs       = 1.0
        self.norm     = 0
        self.dist     = 0
        self.th       = 1*0.06  # threshold repulsivo, multiplo de track del auto track =0.28m
        self.attr_g   = 150.0   # ganancia de atraccion, tunear
        self.etha     = 6.0     # constante repulsiva, tunearla
        self.rep      = 0.0
        self.pmap_res = 0.0

        self.move    = False
        self.pot_active = True
        self.first_loop = True
        self.finish_wp = True
        self.process_ready = False
        self.temp_wp = []
        self.num = 0
        self.size_wp_pp = 0
        self.size_temp_pp = 0
        self.index      = 0

        # Waypoints
        # self.inx_act  = 0
        # self.iny_act  = 0
        self.pot_current = self.start
        self.current_wp = 0
        self.wp_pp = []
        self.grid_index = []
        self.minix = 0
        self.miniy = 0


    def get_points(self):
        usr =  getpass.getuser()
        image = cv2.imread('/home/{}/catkin_ws/src/ros_wall_follower/maps/hector_slam/berlin_5cm.pgm'.format(usr))
        img = cv2.resize(image, None, fx=1.0/self.rs, fy=1.0/self.rs)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
        cv2.imshow("popo", gray)
        self.norm = np.array(thresh/255)
        # print(thresh.shape)
        self.dist = cv2.distanceTransform(thresh, cv2.DIST_L2, 0)
        # print(self.dist.shape)
        # print(type(self.dist))
        # np.set_printoptions(threshold=sys.maxsize)
        # cv2.imshow('popo2', self.dist)
        # print(self.dist)

        # Para visualizar es necesario normalizar, pero distance transform ya nos da dq
        cv2.normalize(self.dist, self.dist, 0, 1.0, cv2.NORM_MINMAX)
        cv2.imshow('Distance Transform Image', self.dist)

        cv2.setMouseCallback("popo", self.click_event)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        self.ready = True

    def click_event(self, event, x, y,flags,param):
        # if event==cv2.EVENT_LBUTTONDOWN:
        #     print("INIT")
        #     start=(y,x)
        if event==cv2.EVENT_RBUTTONDOWN:
            print('FIN')
            point = (y, x)
            self.end.append(point)
            self.index2 = self.index2 + 1

    def potential_process(self):
        minp = float("inf")
        motion = self.get_motion_model()
        for i,_ in enumerate(motion):
            px = int(motion[i][0])
            py = int(motion[i][1])
            self.pmap_res, inx, iny = self.calc_potential_field(self.pot_current, px, py)
            print("POTENCIAL {}: {}, x:{}, y:{}".format(i, self.pmap_res, inx, iny))
            if minp > self.pmap_res:
                minp = self.pmap_res
                self.minix = inx
                self.miniy = iny

        self.grid_index.append([self.minix, self.miniy])
        self.current_wp = (0.05 * (self.minix - self.start[1]), 0.05 * (self.start[0] - self.miniy))
        self.wp_pp.append(self.current_wp)
        self.pot_current = [self.miniy, self.minix]

    def calc_potential_field(self, pot_current, point_x, point_y): 
        pmap_res = 0
        # self.iny_act  = 70 - (round((np.ceil(round(self.current_pos['y'],2) / 0.05) * 0.05) ,2))/0.05
        # self.inx_act  = (round((np.ceil(round(self.current_pos['x'],2) / 0.05) * 0.05) ,2))/0.05  + 232
        index_y = pot_current[0] + point_y
        index_x = pot_current[1] + point_x
        # np.hypot = (pos actual, goal)
        # campo attr
        self.attr = self.attr_g*((index_y - self.end[self.num][0])**2 + (index_x - self.end[self.num][1])**2)
        
        # campo repulsivo
        if index_x >= len(self.dist) or index_y >= len(self.dist) or index_x < 0 or index_y < 0:
            pmap_res = float("inf")
        else:
            dqnorm = self.dist[int(index_y) ,int(index_x)]
            dq = dqnorm / 100.0
            #recorrer dq
            if dq <= self.th:
                self.rep  = self.etha*(1.0/dq - 1.0/self.th)**2
            else:
                self.rep = 0.0
            #resultado instantaneo por celda
            pmap_res = self.attr + self.rep
        # rospy.loginfo("rep:{}, attr:{} ".format(self.rep, self.attr))
        return pmap_res, index_x, index_y

    def get_motion_model(self):
        # checar casillas adyacentes
        motion = []
        for i in [-3, 3]:
            for j in range(-3, 4):
                if i == j:
                    motion.append([i, j])
                else:
                    motion.append([i, j])
                    motion.append([j, i])
        
        return motion

class PurePursuit(PID, Odom, Steering, PotentialField):
    def __init__(self):
        PID.__init__(self, 2.15, 0.00, 0.75)
        Odom.__init__(self)
        Steering.__init__(self)
        PotentialField.__init__(self)

        rospy.init_node("dummy_agent")
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)
        rospy.Timer(rospy.Duration(0.1), self.timerPotentialCallback)

        self.move    
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

    def timerCallback(self, event):
        if self.process_ready:
            if not self.move:
                self.move = True
                self.index = 0
                self.process_ready = False
        if self.finish_wp and self.move:
            self.finish_wp = False
            self.temp_wp = self.wp_pp
            self.size_temp_pp =  self.size_wp_pp
        if self.move:
            self.takeAction()
            self.drive_pub.publish(self.acker_msg)
        # import pdb; pdb.set_trace()
        # rospy.loginfo("TIMER CALLBACK")

    def timerPotentialCallback(self, event):
        self.contProcess()

    def takeAction(self):
        if (np.hypot((self.temp_wp[-1, 0] - self.current_pos['x2']),
                            (self.temp_wp[-1, 1] - self.current_pos['y2']))
                                <= 3.0): # Distancia Umbral
            if not self.pot_active and not self.process_ready:
                rospy.loginfo("UMBRAL ACTIVADO")
                self.pot_active = True
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
        else: 
            pass
        if (self.pot_active and self.ready and self.num < (self.index2)):
            rospy.loginfo("PROCESS")
            self.wp_pp = []
            self.grid_index = []

            self.d = np.hypot(self.pot_current[0] - self.end[self.num][0],
                            self.pot_current[1] - self.end[self.num][1])
            
            s = time.time()
            while(self.d >= 10.0):
                self.potential_process()
                self.d = np.hypot(self.pot_current[0] - self.end[self.num][0],
                            self.pot_current[1] - self.end[self.num][1])
            
            e = time.time()
            print("Tiempo: {}".format(e-s))
            self.num += 1
            self.wp_pp = np.array(self.wp_pp)
            self.size_wp_pp = self.wp_pp.shape[0]
            self.process_ready = True
            self.pot_active = False
        else:
            pass

        # rospy.loginfo("Paso")

if __name__ == "__main__":
    robot = PurePursuit()
    rospy.spin()