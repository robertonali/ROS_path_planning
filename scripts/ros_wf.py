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

class ePID_M(object):
    def __init__(self, kp, ki, kd):
        self.dt       = 0.01
        self.setpoint = 0
        self.gains    = {'Kp': kp, 'Ki': ki, 'Kd': kd}
        self.error    = [0.0, 0.0]
        self.U        = 0.0
        self.steering_output = 0.0
    
    def calculateControl(self, centroid):
        self.setpoint = 0
        self.error[0] = centroid - self.setpoint #self.regions['IZQ']- self.regions['DER']
        Up = self.gains['Kp'] * self.error[0]
        Ui = self.gains['Ki'] * ((self.error[0] - self.error[1]) / 2) * self.dt
        Ud = self.gains['Kd'] * (self.error[0] - self.error[1])/ self.dt

        self.U = Up + Ui + Ud

        #self.steering_output = ((self.max_steering) / (self.gains['Kp'] * 270) * U)
        self.steering_output = min(max(-1., self.U),1)
        rospy.loginfo("Control: {}".format(self.U))
        # sign = 1 if (self.steering_output >= 0) else -1
        # self.steering_output = (sign * self.max_steering) if (abs(self.steering_output) >= self.max_steering) else self.steering_output

        self.error[1] = self.error[0]

class Odom(object):
    def __init__(self):
        self.waypoints_x = list()
        self.waypoints_y = list()
        self.current_pos = {'x': 0.0, 'y': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0,
                'total': 0.0}

class Centroid(object):
    def __init__(self):
        self.centroid              = 0.0
        self.normalized_centroid   = 0.0

class WallFollower(ePID_M, Odom, Centroid):
    def __init__(self):
        # self.pid = PID(1.0, 0.0, 0.0)
        ePID_M.__init__(self, 3, 0.0, 0.01)

        Centroid.__init__(self)

        Odom.__init__(self)

        self.max_steering    = 1.0
        self.vel             = 4.0
        self.acker_msg       = AckermannDriveStamped()
        self.regions         = defaultdict(lambda:float)

        self.segs = 0.0
        
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)

    
    def odomCallback(self, msg):
        self.current_pos['x'] = msg.pose.pose.position.x
        self.current_pos['y'] = msg.pose.pose.position.y
        self.current_vel['x'] = msg.twist.twist.linear.x
        self.current_vel['y'] = msg.twist.twist.linear.y
        self.current_vel['total'] = np.hypot(self.current_vel['x'], self.current_vel['y'])
        # self.z1 = msg.twist.twist.angular.z
        # self.z2 = msg.pose.pose.orientation.z
            
    def laserCallback(self, msg):
        # self.regions = {
        #     'DER'   : np.array(msg.ranges[138:539]),
        #     'IZQ'   : np.array(msg.ranges[541:940])
        # }


        # self.centroid = np.divide(np.sum(np.multiply(msg.ranges, np.arange(1080))),
        #                     np.sum(msg.ranges))

        self.regions = {
            'DER'   : min(min(msg.ranges[138:539]), 30),
            # 'F_DER'  : min(min(msg.ranges[323:420]), 30),
            # 'FRONT': min(min(msg.ranges[421:659]), 30),
            # 'F_IZQ'  : min(min(msg.ranges[661:751]), 30),
            'IZQ'   : min(min(msg.ranges[541:940]), 30),
        }
        self.area=0.0
        self.moment=0.0
        self.centroid=0.0
        for i in range(140,940):
            self.area = self.area+(msg.ranges[i]*.25)
            self.moment = self.moment+(i*msg.ranges[i]*.25)
        self.centroid=self.moment/self.area
        self.normalized_centroid = (self.centroid / 540) -1
        #rospy.loginfo(self.centroid)


    def setCarMovement(self, steering_angle, steering_angle_velocity, speed,
                        acceleration, jerk):
        self.acker_msg = AckermannDriveStamped()
        self.acker_msg.drive.steering_angle = steering_angle
        # self.acker_msg.drive.steering_angle_velocity= steering_angle_velocity
        self.acker_msg.drive.speed = speed
        # self.acker_msg.drive.acceleration = acceleration
        # self.acker_msg.drive.jerk = jerk


        self.drive_pub.publish(self.acker_msg)
    
    def takeAction(self):
        pass
        #self.calcControl()
        # if self.regions['FRONT'] <= 0.2:
        #     self.vel= -3.0
        #     signo = -1 if (self.steering_output >= 0) else 1
        #     self.steering_output = signo
        # elif  self.regions['FRONT'] > 0.2 and self.regions['FRONT'] < 0.9: 
        #     signo = -1 if (self.steering_output >= 0) else 1
        #     self.steering_output = signo if self.vel == -5.0 else self.steering_output * 1
        # elif self.regions['FRONT'] > 0.9 and self.regions['FRONT'] < 1.2:
        #     self.vel = 1.0
        # elif self.regions['FRONT'] > 1.2:
        #     self.vel= 2.0
        # self.setCarMovement(self.steering_output, 0.08, self.vel, 0.0, 0.0)
        # self.pub_drive.publish(self.acker_msg)

    def calcControl(self):
        pass
        # if (self.control_side == "RIGHT"):
        #     self.error[0] = self.setpoint - self.regions['DER']
        # elif (self.control_side == "LEFT"):
        #     self.error[0] = self.regions['IZQ'] - self.setpoint
        # else:
        #     self.setpoint = (self.regions['DER'] + self.regions['IZQ'])/2
        #     self.error[0] = self.setpoint - self.regions['DER']
        # self.setpoint = 0 #(self.regions['DER'] + self.regions['IZQ'])/2
        # self.error[0] = self.centroid2-self.setpoint #self.regions['IZQ']- self.regions['DER']
        # Up = self.gains['Kp'] * self.error[0]
        # Ui = self.gains['Ki']*self.dt * (self.error[0] - self.error[1]) / 2
        # Ud = self.gains['Kd'] * (1 / self.dt) * (self.error[0] - self.error[1])

        # U = Up + Ui + Ud

        # #self.steering_output = ((self.max_steering) / (self.gains['Kp'] * 270) * U)
        # self.steering_output=min(max(-1.,U),1)
        # rospy.loginfo("Output:{} vel:,{}".format(self.steering_output,self.vt))
        # # sign = 1 if (self.steering_output >= 0) else -1
        # # self.steering_output = (sign * self.max_steering) if (abs(self.steering_output) >= self.max_steering) else self.steering_output

        # self.error[1] = self.error[0]
    def timerCallback(self, elapsed):
        # self.takeAction()
        self.calculateControl(self.normalized_centroid)
        self.setCarMovement(self.steering_output, 0.0, self.vel, 0.0, 0.0)
        self.wpdict ={"x": self.waypoints_x,"y": self.waypoints_y}
        self.segs = self.segs + 1
        if self.segs == 30.0:
            self.waypoints_x.append(self.current_pos['x'])
            self.waypoints_y.append(self.current_pos['y'])
            self.segs=0
        DataOutput = pd.DataFrame(self.wpdict)
        DataOutput.to_csv("ros_wall_follower/scripts/coords_mamado.csv" , index=False)
   
        # if (self.control_side == "RIGHT"):
        #     rospy.loginfo("DER: {}, Giro: {}, Error: {}".format(self.regions['DER'], self.acker_msg.drive.steering_angle, self.error[0]))
        # elif (self.control_side == "LEFT"):
        #     rospy.loginfo("IZQ: {}, Giro: {}, Error: {}".format(self.regions['IZQ'], self.acker_msg.drive.steering_angle, self.error[0]))
        # else:
        #     rospy.loginfo("DER: {}, IZQ:{}, Giro: {}, Error: {}".format(self.regions['DER'], self.regions['IZQ'], self.acker_msg.drive.steering_angle, self.error[0]))


if __name__ == "__main__":
    rospy.init_node("dummy_agent")
    robot = WallFollower()
    # robot.main()
    rospy.spin()
