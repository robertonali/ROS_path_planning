#! /usr/bin/env python

import rospy
import math
import numpy as np
import pandas as pd
import cv2
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

class Node(object):
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class AStar(object):
    def __init__(self):
        # Reduction factor
        self.rs     = 4.0
        self.start = (int(np.ceil(70/self.rs)),int(np.ceil(232/self.rs)))
        self.end = list()
        self.index2 = 0
        self.norm = 0
        self.num = 0
        self.wp_pp = 0
        self.size_wp_pp = 0
        self.as_active = True
        self.first_loop = True
        self.move = False

    def get_points(self):
        image = cv2.imread('/home/user/catkin_ws/src/ros_wall_follower/maps/hector_slam/berlin_5cm.pgm')
        # cv2.imshow('ferdinand',image)
        img = cv2.resize(image, None, fx=1.0/self.rs, fy=1.0/self.rs)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)
        # self.norm = np.array(thresh/254)
        cv2.imshow("popo",gray)
        # np.set_printoptions(threshold=sys.maxsize)
        # print(gray)
        self.norm = np.array(thresh/255)

        cv2.setMouseCallback("popo", self.click_event)
        # start = ()
        # end = ()
        # self.start = (70, 232)
        # end = (75, 240)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def solve(self, maze, start, end, fig):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        # Create start and end node
        startNode = Node(None, start)
        startNode.h =  startNode.f = 0
        startNode.g =  startNode.f = 0
        endNode = Node(None, end)
        endNode.h = endNode.f = 0
        endNode.g = endNode.f = 0

        # Initialize both open and closed list
        openList = []
        closedList = []

        # Add the start node
        openList.append(startNode)
        
        n = 1
        # Loop until you find the end
        while len(openList) > 0:

            # Get the current node
            currentNode = openList[0]
            current_index = 0
            for index, item in enumerate(openList):
                if item.f < currentNode.f:
                    currentNode = item
                    current_index = index

            # Pop current off open list, add to closed list
            openList.pop(current_index)
            closedList.append(currentNode)

            # Found the goal
            if currentNode == endNode:
                path = []
                waypoints = []
                current = currentNode
                while current is not None:
                    current_wp = (0.05 * (current.position[1] - 232/self.rs), 0.05 * (70/self.rs - current.position[0]))
                    path.append(current.position)
                    waypoints.append(current_wp)
                    current = current.parent
                
                # mm = genMaze(maze.copy(), start, end, openList, closedList, path[::-1])
                # pltMaze(mm, fig)
                # waypoints = waypoints[::-1]
                # print(waypoints)
                return (path[::-1], waypoints[::-1]) # Return reversed path

            # Generate children
            children = []
            for x, y in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                # Get node position
                nodePosition = (currentNode.position[0] + x, currentNode.position[1] + y)

                # Make sure within range
                if nodePosition[0] > (len(maze) - 1) or nodePosition[0] < 0 or nodePosition[1] > (len(maze[len(maze)-1]) -1) or nodePosition[1] < 0:
                    continue
                # import pdb; pdb.set_trace()
                # Make sure walkable terrain
                if maze[nodePosition[0]][nodePosition[1]] != 0:
                    continue
                # if maze[nodePosition[0]][nodePosition[1]] != 0:
                #     continue

                # Create new node
                newNode = Node(currentNode, nodePosition)

                # Append
                children.append(newNode)

            # Loop through children
            for child in children:

                # Child is on the closed list
                b = False
                for closedChild in closedList:
                    if child == closedChild:
                        b = True #continue
                        break
                if b:
                    continue
                
                # Create the f and g
                # child.g = currentNode.g + 1
                child.g = ((child.position[0] - endNode.position[0]) ** 2) + ((child.position[1] - endNode.position[1]) ** 2)
                child.h = ((child.position[0] - startNode.position[0]) ** 2) + ((child.position[1] - startNode.position[1]) ** 2)
                child.f = child.h + child.g

                # Child is already in the open list
                b = False
                for openNode in openList:
                    if child == openNode and child.h >= openNode.h:
                        b = True #continue
                        break
                if b:
                    continue

                # Add the child to the open list
                openList.append(child)
                
                # if (n % 2) == 1 :
                #     mm = genMaze(maze.copy(), start, end, openList, closedList)
                #     pltMaze(mm, fig)
                
            n = n + 1

    def click_event(self, event,x,y,flags,param):
        # if event==cv2.EVENT_LBUTTONDOWN:
        #     print("INIT")
        #     start=(y,x)
        if event==cv2.EVENT_RBUTTONDOWN:
            print('FIN')
            point = (y, x)
            self.end.append(point)
            self.index2 = self.index2 + 1

    def process(self):
        # wpCSV = []
        self.index2 = self.index2 - 1
        # num = 0
        # while (num <= self.index):
        [pathAstar, wpAstar] = self.solve(self.norm, self.start, self.end[self.num], None)
        self.start = self.end[self.num]
        self.num += 1
        self.wp_pp = 4*np.array(wpAstar)
        self.size_wp_pp = self.wp_pp.shape[0]
        rospy.loginfo(self.size_wp_pp)
            # wpCSV += wpAstar[:-1]
            # print(pathAstar)
            # print(wpAstar)

        # np.savetxt('./scripts/odom_data_A*.csv',wpCSV, delimiter = ",")

class PurePursuit(PID, Odom, Steering, AStar):
    def __init__(self):
        PID.__init__(self, 2.15, 0.00, 0.75)
        Odom.__init__(self)
        Steering.__init__(self)
        AStar.__init__(self)

        rospy.init_node("dummy_agent")
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timerCallback)
        rospy.Timer(rospy.Duration(0.1), self.timerAStarCallback)

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
        if self.move:
            self.takeAction()
            self.drive_pub.publish(self.acker_msg)

    def timerAStarCallback(self, event):
        self.contProcess()

    def takeAction(self):
        if (np.hypot((self.wp_pp[-1, 0] - self.current_pos['x2']),
                            (self.wp_pp[-1, 1] - self.current_pos['y2']))
                                <= 3.0): # Distancia Umbral
            # rospy.loginfo("Umbral")
            self.as_active = True
        else:
            pass

        # if (math.sqrt((self.waypoints[self.wp_index, 0] - self.x2) ** 2 + ((self.waypoints[self.wp_index, 1] - self.y2) ** 2))
        #         <= 1.2) :
        if (np.hypot((self.wp_pp[self.index, 0] - self.current_pos['x2']),
                            (self.wp_pp[self.index, 1] - self.current_pos['y2']))
                                <= 1.2):
            # Go to next target by adding one to the index of the waypoints list
            if self.index < self.size_wp_pp - 1:
                self.index += 1
            else:
                rospy.loginfo(self.index)
                rospy.loginfo(self.size_wp_pp)
                rospy.loginfo("Detente")
                self.move = False
                self.index = 0
        else:
            pass

        self.vel = 1.0 if self.move else 0.0             # self.vel = 1.0 #- abs(self.steering_output) * 3.0/1.22

        self.calcLateralKinematics()
        self.calculateControl(self.crosstrack_error)
        self.delta = np.arctan2( (2 * self.wheelbase * (self.U)), (self.ld)**2 )
        self.steering_output = self.delta # min(max(-self.max_steering, self.delta), self.max_steering)
        self.setCarMovement(self.steering_output, 0.00, self.vel, 0.0, 0.0)
        
        rospy.loginfo("LDCacl:{} , LD: {}, VEL: {} ".format(self.ldcalc, self.ld, self.current_vel['total']))
        rospy.loginfo("Steer: {}, Delta: {}".format(self.steering_output, self.delta))

    def calcLateralKinematics(self):
        # self.ld = math.sqrt((self.waypoints[self.wp_index,0] - self.current_pos['x2']) ** 2 + ((self.waypoints[self.wp_index,1] - self.y2) ** 2))
        self.ld     = np.hypot((self.wp_pp[self.index, 0] - self.current_pos['x2']), (self.wp_pp[self.index, 1] - self.current_pos['y2']))
        self.gama   = math.atan2((self.wp_pp[self.index, 1] - self.current_pos['y2']), (self.wp_pp[self.index, 0] - self.current_pos['x2']))
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
        if (self.as_active and self.ready):
            self.as_active = False
            rospy.loginfo("Process")
            self.process()
            self.index = 0
            self.move = True
        rospy.loginfo("Paso")

if __name__ == "__main__":
    robot = PurePursuit()
    rospy.spin()
