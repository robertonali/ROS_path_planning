#! /usr/bin/env python

import rospy
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
    
    def calculateControl(self, centroid):
        self.setpoint = 0
        self.error[0] = centroid - self.setpoint #self.regions['IZQ']- self.regions['DER']
        Up = self.gains['Kp'] * self.error[0]
        Ui = self.gains['Ki'] * ((self.error[0] + self.error[1]) / 2) * self.dt
        Ud = self.gains['Kd'] * (self.error[0] - self.error[1]) * (1 / self.dt)

        self.U = Up + Ui + Ud

        # self.steering_output = ((self.max_steering) / (self.gains['Kp'] * 270) * U)
        # sign = 1 if (self.steering_output >= 0) else -1
        # self.steering_output = (sign * self.max_steering) if (abs(self.steering_output) >= self.max_steering) else self.steering_output
        # self.steering_output = min(max(-1., self.U),1)
        self.error[1] = self.error[0]
        return min(max(-1.0, self.U), 1.0)

class Orientation(object):
        def __init__(self):
            self.quaternion = list()
            self.euler      = defaultdict(lambda: float)
class Odom(object):
    def __init__(self):
        # self.waypoints_x = list()
        # self.waypoints_y = list()
        self.waypoints   = list()
        self.current_pos = {'x': 0.0, 'y': 0.0}
        self.prev_pos    = {'x': 0.0, 'y': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'total': 0.0}
        self.orientation = Orientation() 
        self.track       = 0.28
        self.wheelbase   = 0.40

class Centroid(object):
    def __init__(self):
        self.centroid             = 0.0
        self.normalized_centroid  = 0.0
        # self.moment             = 0.0
        # self.area               = 0.0

class WallFollower(PID, Odom, Centroid):
    def __init__(self):
        # self.pid = PID(1.0, 0.0, 0.0)
        PID.__init__(self, 2.12, 0.00, 0.42)  # 1.50, 0.4
        # 1.70, 0.4
        # 2.12, 0.42

        Centroid.__init__(self)

        Odom.__init__(self)

        self.max_steering    = 1.0
        self.steering_output = 0.0
        self.vel             = 5.0
        self.acker_msg       = AckermannDriveStamped()
        self.regions         = defaultdict(lambda: float)

        rospy.init_node("dummy_agent")

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
        
        self.orientation.quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (self.orientation.euler['roll'], self.orientation.euler['roll'],
            self.orientation.euler['yaw']) = transformations.euler_from_quaternion(self.orientation.quaternion)
            
    def laserCallback(self, msg):
        # self.regions = {
        #     'DER'   : np.array(msg.ranges[138:539]),
        #     'IZQ'   : np.array(msg.ranges[541:940])
        # }
        # for i in range(140,940):
        #     self.area = self.area + (msg.ranges[i] * .25)
        #     self.moment = self.moment + (i * msg.ranges[i] * .25)
        # self.centroid = self.moment / self.area
        self.centroid = np.divide(np.sum(np.multiply(msg.ranges[140:940], np.arange(140, 940))),
                    np.sum(msg.ranges[140:940]))
        self.normalized_centroid = ((self.centroid / 400) - 1.35)
        # rospy.loginfo(self.normalized_centroid)

    def setCarMovement(self, steering_angle, steering_angle_velocity, speed,
                        acceleration, jerk):
        self.acker_msg.drive.steering_angle = steering_angle
        # self.acker_msg.drive.steering_angle_velocity= steering_angle_velocity
        self.acker_msg.drive.speed = speed
        # self.acker_msg.drive.acceleration = acceleration
        # self.acker_msg.drive.jerk = jerk

    def timerCallback(self, event):
        self.steering_output = self.calculateControl(self.normalized_centroid)
        self.vel = 5.0 - abs(self.steering_output) * 3.0/1.22
        self.setCarMovement(self.steering_output, 0.0, self.vel, 0.0, 0.0)
        self.drive_pub.publish(self.acker_msg)
        self.getWaypoints()
        
        #rospy.loginfo("Steer: {}, Error:{} vel:{}, Centroid: {}".format(self.steering_output,
        #                self.error[0], self.current_vel['total'], self.centroid))

    def getWaypoints(self):
        if ( (np.hypot((self.current_pos['x'] - self.prev_pos['x']),
                        (self.current_pos['y'] - self.prev_pos['y']))) >= (self.wheelbase * 0.1) ):
            self.prev_pos['x'] = self.current_pos['x']                          # Change value of prev x on dict
            self.prev_pos['y'] = self.current_pos['y']                          # Change value of prev y on dict
            waypoint_x2 = self.current_pos['x'] + ((self.wheelbase / 2) * np.cos(self.orientation.euler['yaw'])) # Get waypoint to the rear x axis
            waypoint_y2 = self.current_pos['y'] + ((self.wheelbase / 2) * np.sin(self.orientation.euler['yaw'])) # Get waypoint to the rear y axis
            self.waypoints.append( [waypoint_x2, waypoint_y2,
                                    self.current_vel['total']] )                # List appending x data
            np.savetxt('./ros_wall_follower/scripts/odom_data_corto.csv', self.waypoints, delimiter = ",")


if __name__ == "__main__":
    robot = WallFollower()
    rospy.spin()