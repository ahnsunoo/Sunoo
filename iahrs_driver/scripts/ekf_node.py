#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math
from scipy.spatial.transform import Rotation as R

def imu_callback(msg):
    # 쿼터니언 -> 오일러 변환 (scipy 사용)
    q = msg.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    r = R.from_quat(quaternion)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)

    # yaw 정규화: -180 ~ 180
    new_yaw = (yaw) % 360 - 180

    yaw_pub.publish(Float32(data=yaw))

def main():
    rospy.init_node('imu_yaw_extractor')
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    global yaw_pub
    yaw_pub = rospy.Publisher("/imu/yaw", Float32, queue_size=10)
    rospy.loginfo("Yaw extractor started (Python 3 + scipy)")
    rospy.spin()

if __name__ == "__main__":
    main()
