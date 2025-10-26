#!/usr/bin/python3

import rospy
import math 
import time
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Twist

def steering_save_pub(data):
    Motor_sensor_value = data.linear.x
    Mapping_steering_angle_Mearsure = data.linear.y
    Subscribed_steering_angle = data.linear.z
    Neural_steering_angle = data.angular.x
    steering_angle = np.zeros([4])

    steering_angle[0], steering_angle[1], steering_angle[2] ,steering_angle[3] = Motor_sensor_value, Mapping_steering_angle_Mearsure, Subscribed_steering_angle, Neural_steering_angle
    steering_angle_list.append(steering_angle)
    df = pd.DataFrame(steering_angle_list, columns=['Motor_sensor_value', 'Mapping_steering_angle_Mearsure', 'Subscribed_steering_angle', 'Neural_steering_angle'])
    df.to_csv('/home/ubuntu/results/arduino_measure/' + str(trial) + '.csv')


rospy.init_node("steering_save_node")
Steering = rospy.Subscriber( "/steering_angle", Twist, callback = steering_save_pub)
steering_angle_list = []
date = str(datetime.now().date()) + " - " + str(datetime.now().time())
trial = date

if __name__ == "__main__":
    try:
        rospy.spin()
        # rospy.rate(0.05)
        time.sleep(100)
    except KeyboardInterrupt:
        print("shutdown")