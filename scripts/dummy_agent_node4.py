#!/usr/bin/env python

# all the used libraries are imported
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# The class is defined 
class Agent(object):
    def __init__(self):
        
        # wall follower variables
        self.r = 0
        self.l = 0
        self.error = 0
        self.error_1 = 0
        self.dt = 0.01

        self.pid = 0

        # odometry variables are created
        self.current_x = 0
        self.current_y = 0
        self.current_v_x = 0
        self.current_v_y = 0
        self.current_v = (self.current_v_x**2 + self.current_v_y**2) ** 0.5
        self.current_roll = 0
        self.current_pitch = 0
        self.current_yaw = 0

        # publishers and subscribers
        self.odom_list = list()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

    # this function is accessed every 0.01 seconds
    def timer_callback(self, elapsed):
        self.error_1 = self.error 

        # the difference between the left and right measurements is computed
        self.error = self.l - self.r

        # pid variables
        p = 0.24 * self.error
        d = 0.12 * (self.error - self.error_1) / self.dt
        self.pid = p + d

        # linearization of the velocity according to the pid
        v = 5.0 - abs(self.pid) * 3.0/1.22

        # speed and steering angle are published
        drive = AckermannDriveStamped()
        drive.drive.speed = v
        drive.drive.steering_angle = self.pid
        self.drive_pub.publish(drive)
        rospy.loginfo("vel:{}".format(v))

    def scan_callback(self, scan_msg):
        # left and right measurements are aquired
        r_segment = np.array(scan_msg.ranges[140:540])
        l_segment = np.array(scan_msg.ranges[-540:-140])
        
        # a mean is computed for each side
        self.r = np.mean(r_segment)
        self.l = np.mean(l_segment)

    def odom_callback(self, odom_msg):
        # orientation data is acquired - data is transformed from quat to euler matrix
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.yaw_1 = self.current_yaw
        (self.current_roll, self.current_pitch, self.current_yaw) = euler_from_quaternion (orientation_list)

        # linear odometry is acquired
        self.current_x = odom_msg.pose.pose.position.x + 0.17*np.cos(self.current_yaw)
        self.current_y = odom_msg.pose.pose.position.y + 0.17*np.sin(self.current_yaw)
        self.current_v_x = odom_msg.twist.twist.linear.x
        self.current_v_y = odom_msg.twist.twist.linear.y
        self.current_v = (self.current_v_x**2 + self.current_v_y**2) ** 0.5
        self.current_v_th = np.arctan2(self.current_v_y, self.current_v_x)

        # measurements are stored in a dat file
        self.odom_list.append([self.current_x, self.current_y, self.current_v])
        np.savetxt('odom_data.dat', self.odom_list)

if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    dummy_agent = Agent()
    rospy.spin()