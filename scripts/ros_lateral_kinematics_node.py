#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from collections import defaultdict
from tf import transformations
import math
import pandas as pd
import numpy as np

class WallFollower(object):
    def __init__(self):
        self.acker_msg = AckermannDriveStamped()
        self.regions = defaultdict(lambda:float)
        self.gains = {
            'Kp': 0.2,#1.0,
            'Ki': 0.00,#0.0025,
            'Kd': 0.00,#0.0005,
        }
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.laserCallback, queue_size=1)
        self.max_steering = 1.22
        self.setpoint = 0.0
        self.dt = 0.01
        self.error = [0.0, 0.0]
        self.u = 0.0
        self.steering_output = 0.0
        self.vel = 1.0
        self.track = .28
        self.wbase = .40
        self.waypoints = np.genfromtxt('./ros_wall_follower/scripts/odom_data_corto.csv', delimiter=',')
        self.wp_size = self.waypoints.size
        self.wp_index = 0
        self.yaw = 0.0
        self.ld = 0.0
        self.ldcalc=0.0
        self.kd = 0.40
        self.x = 0.0
        self.y = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.gama = 0.0
        self.alpha = 0.0
        self.delta = 0.0
        self.vx = 0
        self.vy = 0
        self.vt = np.hypot(self.vx,self.vy)
        self.bucio = True

        rospy.init_node('dummy_agent')
        self.pub_drive = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_= rospy.Subscriber("/odom", Odometry,self.odomCallback,queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)
    
    def odomCallback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            )
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vt = np.hypot(self.vx,self.vy)
        self.x2 = self.x - ((self.wbase / 2) * np.cos(self.yaw))
        self.y2 = self.y - ((self.wbase / 2) * np.sin(self.yaw)) 

    def laserCallback(self, msg):
        self.regions = {
            'DER'   : min(min(msg.ranges[138:539]), 30),
            'IZQ'   : min(min(msg.ranges[541:940]), 30),
        }

    def setCarMovement(self, steering_angle, steering_angle_velocity, speed,
                        acceleration, jerk):
        self.acker_msg.drive.speed = speed
        # self.acker_msg.drive.acceleration = acceleration
        self.acker_msg.drive.steering_angle_velocity= steering_angle_velocity
        self.acker_msg.drive.steering_angle = steering_angle
        # self.acker_msg.drive.jerk = jerk
    
    def takeAction(self):
        if (self.bucio==True):
            #self.ld = math.sqrt((self.waypoints[self.wp_index,0] - self.x2) ** 2 + ((self.waypoints[self.wp_index,1] - self.y2) ** 2))
            self.ld = np.hypot((self.waypoints[self.wp_index,0] - self.x2), (self.waypoints[self.wp_index,1] - self.y2))
            self.gama = math.atan2((self.waypoints[self.wp_index,1] - self.y2), (self.waypoints[self.wp_index,0] - self.x2))
            self.alpha = self.gama - self.yaw
            self.ldcalc=(self.kd)*(self.vt) 
            self.delta = math.atan2((2 * self.wbase * math.sin(self.alpha)), (self.ldcalc))
            #rospy.loginfo(self.delta)
            self.bucio = False
        else:
            if (1.2>=math.sqrt((self.waypoints[self.wp_index,0] - self.x2) ** 2 + ((self.waypoints[self.wp_index,1] - self.y2) ** 2))) :
                self.bucio = True
                self.wp_index += 1
            else:
                self.bucio = False
        self.crosstr_error=self.ldcalc*math.sin(self.alpha)
        rospy.loginfo("LDCacl:{} , LD: {}, VEL: {} ".format(self.ldcalc,self.ld,self.vt))
        self.crosstr_error_norm=self.crosstr_error*(2/(self.ld)**2)
        #self.calcControl()
        self.steering_output = self.delta
        rospy.loginfo("Steer: {}, ".format(self.steering_output))
        self.setCarMovement(self.steering_output, 0.00, self.waypoints[self.wp_index,2], 0.0, 0.0)

    def calcControl(self):
        # self.setpoint = 540 
        self.error[0] = self.crosstr_error
        Up = self.gains['Kp'] * self.error[0]
        Ui = self.gains['Ki']*self.dt * (self.error[0] - self.error[1]) / 2
        Ud = self.gains['Kd'] * (1 / self.dt) * (self.error[0] - self.error[1])

        U = Up + Ui + Ud

        self.steering_output = math.atan(U) # (2/(self.ld)**2)
        #self.steering_output = min(max(-1.23,U),1.23)

        # sign = 1 if (self.steering_output >= 1.3) else -1
        # self.steering_output=-1* if (U >= 0) else self.steering_output

        self.error[1] = self.error[0]

    def timerCallback(self, event):
        self.takeAction()
        self.pub_drive.publish(self.acker_msg)
       

if __name__ == "__main__":
    robot = WallFollower()
    # robot.main()
    rospy.spin()
