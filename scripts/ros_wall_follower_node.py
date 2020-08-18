#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from collections import defaultdict
from tf import transformations
import math
import pandas as pd

class WallFollower(object):
    def __init__(self):
        self.segs=0
        self.waypoints_x = []
        self.waypoints_y = []
        
        self.acker_msg = AckermannDriveStamped()
        self.regions = defaultdict(lambda:float)
        self.gains = {
            'Kp': 1.0,
            'Ki': 0.0025,
            'Kd': 0.0005,
        }
        self.max_steering = 1
        self.setpoint = 2.0
        self.dt = 0.1
        self.error = [0.0, 0.0]
        self.u = 0.0
        self.steering_output = 0.0
        self.pa_donde=''
        self.vel = 2.0

        rospy.init_node("ros_wall_follower_node")
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.laserCallback, queue_size=1)
        self.pub_drive = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_= rospy.Subscriber("/odom", Odometry,self.odomCallback,queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)
    
    def odomCallback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
            
    def laserCallback(self, msg):
        self.regions = {
            'DER'   : min(min(msg.ranges[230:250]), 30),
            'F_DER'  : min(min(msg.ranges[251:420]), 30),
            'FRONT': min(min(msg.ranges[421:660]), 30),
            'F_IZQ'  : min(min(msg.ranges[661:784]), 30),
            'IZQ'   : min(min(msg.ranges[785:1079]), 30),
        }
        #rospy.loginfo(regions)
        # self.angle_increment = msg.angle_increment
        # self.steps = (msg.angle_max - msg.angle_min) / self.angle_increment
        # rospy.loginfo("Step is {}, Distances per region {}".format(self.steps, self.regions['Der']))

    def setCarMovement(self, steering_angle, steering_angle_velocity, speed,
                        acceleration, jerk):
        self.acker_msg.drive.speed = speed
        # self.acker_msg.drive.acceleration = acceleration
        self.acker_msg.drive.steering_angle_velocity= steering_angle_velocity
        self.acker_msg.drive.steering_angle = steering_angle
        # self.acker_msg.drive.jerk = jerk
    
    def takeAction(self):
        self.calcControl()
        if self.regions['FRONT'] <= 0.2:
            self.vel= -5.0
            signo = -1 if (self.steering_output >= 0) else 1
            self.steering_output = signo
        elif self.regions['FRONT']< 0.9 and self.regions['FRONT']> 0.2: 
            signo = -1 if (self.steering_output >= 0) else 1
            self.steering_output = signo if self.vel == -5.0 else self.steering_output * 1
        elif self.regions['FRONT']< 1.4 and self.regions['FRONT']> 0.9:
            self.vel = 1.0
        elif self.regions['FRONT'] > 1.4:
            self.vel= 2.0
        self.setCarMovement(self.steering_output, 0.08,self.vel, 0.0, 0.0)
        # rospy.loginfo(self.pa_donde)
        #self.pub_drive.publish(self.acker_msg)

    def calcControl(self):
        self.error[0] = self.setpoint - self.regions['DER']
        Up = self.gains['Kp'] * self.error[0]
        Ui = self.gains['Ki'] * (self.error[0] - self.error[1]) / 2
        Ud = self.gains['Kd'] * (1 / self.dt) * (self.error[0] - self.error[1])

        U = Up + Ui + Ud

        self.steering_output = ((self.max_steering) / (self.gains['Kp'] * 3) * U)

        sign = 1 if (self.steering_output >= 0) else -1
        self.steering_output = (sign * self.max_steering) if (abs(self.steering_output) >= self.max_steering) else self.steering_output

        self.error[1] = self.error[0]

    def timerCallback(self, event):
        self.takeAction()
        #rospy.loginfo("x:{}, y:{}, ".format(self.x,self.y))
        rospy.loginfo("waypoints:{},{} ".format(self.waypoints_x,self.waypoints_y))
        rospy.loginfo("{}, {}, {}, {},".format(self.acker_msg.drive.steering_angle, self.acker_msg.drive.speed, self.regions['IZQ'],self.regions['FRONT']))
        self.wpdict ={"x":self.waypoints_x,"y":self.waypoints_y}
        self.pub_drive.publish(self.acker_msg)
        self.segs = self.segs+1
        if self.segs == 20:
            self.waypoints_x.append(self.x)
            self.waypoints_y.append(self.y)
            self.segs=0
        DataOutput = pd.DataFrame(self.wpdict)
        DataOutput.to_csv("ros_wall_follower/scripts/coords.csv" , index=False)




if __name__ == "__main__":
    robot = WallFollower()
    # robot.main()
    rospy.spin()
