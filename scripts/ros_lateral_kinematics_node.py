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
        self.acker_msg = AckermannDriveStamped()
        self.regions = defaultdict(lambda:float)
        self.gains = {
            'Kp': 1.1,#1.0,
            'Ki': 0.0025,#0.0025,
            'Kd': 0.0005,#0.0005,
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
        self.waypoints = pd.read_csv("./coords.csv", sep=',')
        self.wp_size = self.waypoints.size
        self.wp_index = 0
        self.yaw = 0.0
        self.ld = 0.0
        self.x = 0.0
        self.y = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.gama = 0.0
        self.alpha = 0.0
        self.delta = 0.0
        self.bucio = True

        rospy.init_node("ros_lateral_kinematics_node")
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
        self.x2 = self.x - ((self.wbase / 2) * math.cos(self.yaw))
        self.y2 = self.y - ((self.wbase / 2) * math.sin(self.yaw))
        rospy.loginfo("x2:{}".format(self.x2))
        rospy.loginfo("y2:{}".format(self.x2))
        if (self.bucio):
            rospy.loginfo(self.x)
            rospy.loginfo(self.y)

            rospy.loginfo(self.x2)
            rospy.loginfo(self.y2)
            self.ld = math.sqrt((self.waypoints['x'][self.wp_index] - self.x2) ** 2 + ((self.waypoints['y'][self.wp_index] - self.y2) ** 2))
            rospy.loginfo(self.ld)
            rospy.loginfo(self.yaw)
            self.gama = math.atan2((self.waypoints['y'][self.wp_index] - self.y2), (self.waypoints['x'][self.wp_index] - self.x2))
            rospy.loginfo(self.gama)
            self.alpha = self.gama - self.yaw
            rospy.loginfo(self.alpha)
            self.delta = math.atan2((2 * self.wbase * math.sin(self.alpha)), (self.ld))
            rospy.loginfo(self.delta)
            self.bucio = False
        else:
            if (0.05>=math.sqrt((self.waypoints['x'][self.wp_index] - self.x2) ** 2 + ((self.waypoints['y'][self.wp_index] - self.y2) ** 2))) :
                self.bucio = True
                self.wp_index += 1
            else:
                self.bucio = False
        rospy.loginfo(self.bucio)
        # self.calcControl()
        self.steering_output = self.delta
        rospy.loginfo("Steer: {}, ".format(self.steering_output))
        self.setCarMovement(self.steering_output, 0.08, self.vel, 0.0, 0.0)

    def calcControl(self):
        self.setpoint = 540 
        self.error[0] = self.centroid-self.setpoint 
        Up = self.gains['Kp'] * self.error[0]
        Ui = self.gains['Ki']*self.dt * (self.error[0] - self.error[1]) / 2
        Ud = self.gains['Kd'] * (1 / self.dt) * (self.error[0] - self.error[1])

        U = Up + Ui + Ud

        self.steering_output = ((self.max_steering) / (self.gains['Kp'] * 270) * U)

        sign = 1 if (self.steering_output >= 0) else -1
        self.steering_output = (sign * self.max_steering) if (abs(self.steering_output) >= self.max_steering) else self.steering_output

        self.error[1] = self.error[0]

    def timerCallback(self, event):
        self.takeAction()
        self.pub_drive.publish(self.acker_msg)
       

if __name__ == "__main__":
    robot = WallFollower()
    # robot.main()
    rospy.spin()
