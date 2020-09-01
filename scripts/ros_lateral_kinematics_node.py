#! /usr/bin/env python

import rospy
import math
import numpy as np
import pandas as pd
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
        # self.setpoint = setpoint # -1, 1
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
        self.waypoints   = np.genfromtxt('./ros_wall_follower/scripts/csv/odom_data.csv', delimiter=',')
        self.current_pos = {'x': 0.0, 'y': 0.0, 'x2': 0.0, 'y2': 0.0}
        self.prev_pos    = {'x': 0.0, 'y': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'total': 0.0}
        self.orientation = Orientation() 
        self.track       = 0.28
        self.wheelbase   = 0.40
        self.index       = 0
        
class Steering(object):
    def __init__(self):
        self.max_steering    = 1.22
        self.steering_output = 0.0

class PurePursuit(PID, Odom, Steering):
    def __init__(self):
        # self.pid = PID(1.0, 0.0, 0.0)
        PID.__init__(self, 2.15, 0.00, 0.75)
        Odom.__init__(self)
        Steering.__init__(self)

        rospy.init_node("dummy_agent")
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)

        self.acker_msg       = AckermannDriveStamped()
        # self.regions         = defaultdict(lambda: float)
        # LateralKinematics variables
        self.ld     = 0.0
        self.ldcalc = 0.0
        self.kdd    = 0.30   # 0.40
        self.alpha  = 0.0    # Angle between actual car position and waypoint ahead (gama - yaw)
        self.gamma  = 0.0    # Angle between 0deg and waypoint ahead
        self.delta  = 0.0    # Turning wheels angle.
        self.crosstrack_error = 0.0
        self.orientation.euler['yaw'] = 0.0
        #
        self.next_target = True
    
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
        # self.centroid = np.divide(np.sum(np.multiply(msg.ranges[140:940], np.arange(140, 940))),
        #             np.sum(msg.ranges[140:940]))
        # self.normalized_centroid = ((self.centroid / 400) - 1.35)
        pass

    def timerCallback(self, event):
        self.takeAction()
        self.drive_pub.publish(self.acker_msg)
        
    def takeAction(self):
        if (self.next_target):
            self.calcLateralKinematics() # kdd = 4
            self.next_target = False
        # elif (math.sqrt((self.waypoints[self.wp_index, 0] - self.x2) ** 2 + ((self.waypoints[self.wp_index, 1] - self.y2) ** 2))
        #         <= 1.2) :
        elif (np.hypot((self.waypoints[self.index, 0] - self.current_pos['x2']),
                            (self.waypoints[self.index, 1] - self.current_pos['y2']))
                                <= 1.2):
            self.next_target = True
            self.index += 1
        else:
            pass
            
        # self.calcLateralKinematics()
        self.calculateControl(self.crosstrack_error)
        self.delta = np.arctan2( (2 * self.wheelbase * (self.U)), (self.ld)**2 )

        self.steering_output = self.delta # min(max(-self.max_steering, self.delta), self.max_steering)
        self.setCarMovement(self.steering_output, 0.00, self.waypoints[self.index, 2], 0.0, 0.0)
        
        rospy.loginfo("LDCacl:{} , LD: {}, VEL: {} ".format(self.ldcalc, self.ld, self.current_vel['total']))
        rospy.loginfo("Steer: {}, Delta: {}".format(self.steering_output, self.delta))

    def calcLateralKinematics(self):
        # self.ld = math.sqrt((self.waypoints[self.wp_index,0] - self.current_pos['x2']) ** 2 + ((self.waypoints[self.wp_index,1] - self.y2) ** 2))
        self.ld     = np.hypot((self.waypoints[self.index, 0] - self.current_pos['x2']), (self.waypoints[self.index, 1] - self.current_pos['y2']))
        self.gama   = math.atan2((self.waypoints[self.index, 1] - self.current_pos['y2']), (self.waypoints[self.index, 0] - self.current_pos['x2']))
        self.alpha  = self.gama - self.orientation.euler['yaw']
        self.ldcalc = (self.kdd) * (self.current_vel['total']) 
        self.delta  = math.atan2((2 * self.wheelbase * math.sin(self.alpha)), (self.ldcalc))
        self.crosstrack_error = self.ldcalc * math.sin(self.alpha)
        # self.crosstr_error_norm = self.crosstrack_error * (2 / (self.ld)**2)
    
    def setCarMovement(self, steering_angle, steering_angle_velocity, speed,
                        acceleration, jerk):
        self.acker_msg.drive.steering_angle = steering_angle
        # self.acker_msg.drive.steering_angle_velocity= steering_angle_velocity
        self.acker_msg.drive.speed = speed
        # self.acker_msg.drive.acceleration = acceleration
        # self.acker_msg.drive.jerk = jerk


if __name__ == "__main__":
    robot = PurePursuit()
    rospy.spin()
