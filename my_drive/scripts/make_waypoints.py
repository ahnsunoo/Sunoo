#!/usr/bin/env python3

import rospy
import math 
import time
import pandas as pd
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import math
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
import rospy
import math
import time
import os
from datetime import datetime
import pandas as pd
from std_msgs.msg import Float64, Float32, Int32
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class make_waypoints():
    def __init__(self):
        rospy.loginfo("initializing") 
        rospy.init_node('make_waypoints_using_gps')
        self.xyz_sub = rospy.Subscriber('/gps/utm_pos1', Pose2D, callback=self.MakeWaypointCallback)

        self.waypoint_xy_list = []
        self.waypoint_interval = 2
        self.count = 0
        
    def MakeWaypointCallback(self, data):
        self.count=self.count+1
        print(self.count)
        if self.count % self.waypoint_interval == 0 and self.count > 16:
            xy_array = np.zeros([2])
            

            current_x = data.x
            current_y = data.y

            
            xy_array[0], xy_array[1] = current_x, current_y

            self.waypoint_xy_list.append(xy_array)
            df = pd.DataFrame(self.waypoint_xy_list, columns=['x', 'y']) 
            df.to_csv('/home/icas/catkin_ws/plus_mijagong.csv')
            
        
a = make_waypoints()
rospy.spin()
