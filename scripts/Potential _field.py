#! /usr/bin/env python

import rospy
import math
import numpy as np
import cv2
import sys
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from collections import defaultdict
from tf import transformations
import time

class Steering(object):
    def __init__(self):
        self.max_steering    = 1.22
        self.steering_output = 0.0

class Odom(object):
    def __init__(self):
        self.current_pos = {'x': 0.0, 'y': 0.0, 'x2': 0.0, 'y2': 0.0}
        self.prev_pos    = {'x': 0.0, 'y': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'total': 0.0}
        self.track       = 0.28
        self.wheelbase   = 0.40
        self.index = 0

class PotentialField(Odom,Steering):
    def __init__(self):
        Odom.__init__(self)
        Steering.__init__(self)
        self.end      = (70,300)
        self.rs       = 1.0
        self.norm     = 0
        self.dist     = 0
        self.dt       = 0.01
        self.th       = 3*self.track  #threshold repulsivo, multiplo de track del auto track =0.28m
        self.attr_g   = 5.0     #ganancia de atraccion, tunear
        self.etha     = 100.0   #constante repulsiva, tunearla
        self.rep      = 0.0
        self.pmap_res = 0.0
        self.ready    = False
        self.inx_act  = 0
        self.iny_act  = 0

        rospy.init_node("dummy_agent")
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)
        self.acker_msg       = AckermannDriveStamped()
        
    def timerCallback(self, event):
        if self.ready:
            minp = float("inf")
            motion = self.get_motion_model()
            for i,_ in enumerate(motion):
                px = int(motion[i][0])
                py = int(motion[i][1])
                self.pmap_res, inx, iny = self.calc_potential_field(px, py)
                if minp > self.pmap_res:
                    minp = self.pmap_res
                    minix = inx
                    miniy = iny
            
            angle = np.arctan2(self.iny_act - miniy, self.inx_act - minix)             
            if angle > np.pi/2:
                self.steering_output = angle - np.pi/2
            elif angle <= np.pi/2:
                self.steering_output = -1*(np.pi/2 - angle)
            #self.steering_output = angle - np.pi/2
            rospy.loginfo("minx:{}, miny:{} , steering:{}, inx:{} ,iny{}, angle {}"
            .format(minix, miniy,self.steering_output,self.inx_act,self.iny_act, angle))
            self.setCarMovement(self.steering_output, 0.00, 1.0, 0.0, 0.0)
        else:
            self.setCarMovement(0.0, 0.00, 0.0, 0.0, 0.0)
        self.drive_pub.publish(self.acker_msg)

        # if self.move:
        #     self.takeAction()
        #     self.drive_pub.publish(self.acker_msg)
        #      self.gradient()
        #       self.control()
        # pass

    def odomCallback(self, msg):
        self.current_pos['x'] = msg.pose.pose.position.x
        self.current_pos['y'] = msg.pose.pose.position.y

    def get_points(self):
        image = cv2.imread('/home/user/catkin_ws/src/ros_wall_follower/maps/hector_slam/berlin_5cm.pgm')
        img = cv2.resize(image, None, fx=1.0/self.rs, fy=1.0/self.rs)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
        cv2.imshow("popo",gray)
        self.norm = np.array(thresh/255)
        print(thresh.shape)
        self.dist = cv2.distanceTransform(thresh, cv2.DIST_L2, 0)
        print(self.dist.shape)
        # print(type(self.dist))
        
        # np.set_printoptions(threshold=sys.maxsize)
        # # cv2.imshow('popo2', self.dist)
        # print(self.dist)
        # Para visualizar es necesario normalizar, pero distance transform ya nos da dq
        # cv2.normalize(self.dist, self.dist, 0, 1.0, cv2.NORM_MINMAX)
        # cv2.imshow('Distance Transform Image', self.dist)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        self.ready = True

    def calc_potential_field(self, point_x, point_y): 
        pmap_res = 0
        self.iny_act  = 70 - (round((np.ceil(round(self.current_pos['y'],2) / 0.05) * 0.05) ,2))/0.05
        self.inx_act  = (round((np.ceil(round(self.current_pos['x'],2) / 0.05) * 0.05) ,2))/0.05  + 232
        # Add actual point
        index_y = self.iny_act + point_y
        index_x = self.inx_act + point_x
        
        # np.hypot = (pos actual, goal)
        # campo attr
        self.attr = self.attr_g*((index_y - self.end[0])**2 + (index_x - self.end[1])**2)
        
        # campo repulsivo
        if index_x >= len(self.dist) or index_y >= len(self.dist) or index_x < 0 or index_y < 0:
            pmap_res = float("inf")
        else:
            dqnorm = self.dist[int(index_y) ,int(index_x)]
            dq=dqnorm/100.0
            #recorrer dq
            if dq <= self.th:
                self.rep  = self.etha*(1.0/dq - 1.0/self.th)**2
            else:
                self.rep = 0.0
            #resultado instantaneo por celda
            pmap_res = self.attr + self.rep
        rospy.loginfo("rep:{}, attr:{} ".format(self.rep, self.attr))
        return pmap_res, index_x, index_y

    def get_motion_model(self):
        # checar casillas adyacentes
        motion = [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]]
        
        return motion

    def setCarMovement(self, steering_angle, steering_angle_velocity, speed,
                        acceleration, jerk):
        self.acker_msg.drive.steering_angle = steering_angle
        # self.acker_msg.drive.steering_angle_velocity= steering_angle_velocity
        self.acker_msg.drive.speed = speed
        # self.acker_msg.drive.acceleration = acceleration
        # self.acker_msg.drive.jerk = jerk

if __name__ == "__main__":
    robot = PotentialField()
    robot.get_points()
    rospy.spin()