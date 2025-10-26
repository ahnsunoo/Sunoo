#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 파일명: ekf.py (게이트 로직 삭제 버전)

import rospy
import numpy as np
from numpy.linalg import inv
from math import sin, cos, atan2
from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

# Q: 프로세스 노이즈 공분산 행렬
Q = np.diag([
    0.05,
    0.05,
    np.deg2rad(1.5),
    1.0e-4
])**2

# R: 측정 노이즈 공분산 (GPS의 불확실성)
R = np.diag([0.1, 0.1])**2                  

class EKFNode:
    def __init__(self):
        rospy.init_node('ekf_sensor_fusion_node')

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gps/utm_pos1', Pose2D, self.gps_callback)
        
        self.ekf_pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)

        self.x = np.zeros((4, 1)) # 상태 벡터 [x, y, yaw, gyro_bias]
        
        # 위치에 대한 초기 불확실성을 매우 크게 설정
        self.P = np.diag([
            1000.0,    # x에 대한 불확실성 (큰 값)
            1000.0,    # y에 대한 불확실성 (큰 값)
            np.pi**2,  # yaw에 대한 불확실성 (큰 값, 약 180도)
            0.1        # gyro_bias에 대한 불확실성
        ])
        
        self.last_time = None
        
        rospy.loginfo("4-State EKF Node Initialized (Gate Removed).")

    def imu_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time
        
        omega_measured = msg.angular_velocity.z
        
        self.predict(omega_measured, dt)
        self.publish_odometry()

    def predict(self, omega_measured, dt):
        yaw = self.x[2, 0]
        gyro_bias = self.x[3, 0]
        
        true_omega = omega_measured - gyro_bias
        
        self.x[2] = yaw + true_omega * dt
        self.x[3] = gyro_bias
        self.x[2] = self.normalize_angle(self.x[2])

        F = np.eye(4)
        F[2, 3] = -dt

        self.P = F @ self.P @ F.T + Q

    def gps_callback(self, msg):
        z = np.array([[msg.x], [msg.y]])
        
        H = np.array([[1.0, 0.0, 0.0, 0.0], 
                      [0.0, 1.0, 0.0, 0.0]])
        
        # [삭제] 마할라노비스 거리 기반 Outlier 제거 (게이트 로직) 부분 삭제
        
        self.update(z, H) # update 함수 호출 방식 변경
        rospy.loginfo_once("First GPS update received.")

    def update(self, z, H): # update 함수 인자 변경
        y = z - (H @ self.x)
        S = H @ self.P @ H.T + R
        
        try:
            K = self.P @ H.T @ inv(S)
        except np.linalg.LinAlgError:
            rospy.logwarn("Failed to compute inverse of S matrix. Skipping GPS update.")
            return

        self.x = self.x + (K @ y)
        self.x[2] = self.normalize_angle(self.x[2])
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        
        odom_msg.pose.pose.position.x = self.x[0, 0]
        odom_msg.pose.pose.position.y = self.x[1, 0]
        
        yaw = self.x[2, 0]
        q = quaternion_from_euler(0, 0, yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        self.ekf_pub.publish(odom_msg)

    def normalize_angle(self, angle):
        return atan2(sin(angle), cos(angle))


if __name__ == '__main__':
    try:
        ekf_node = EKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
