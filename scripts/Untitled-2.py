#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from collections import defaultdict
from tf import transformations
import math
import pandas as pd

su=0
for i in range(1079):
    su = su +.25
    print(su)

    