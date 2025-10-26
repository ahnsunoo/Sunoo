#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Noetic - 2D EKF (GPS Pose2D + IMU)  — /imu/yaw 비사용 버전
- 상태 x = [x, y, psi, v]^T
- 예측 입력 u: r = /imu/data.angular_velocity.z (rad/s)
- 측정 z_gps: [x, y] from /gps/utm_pos1 (geometry_msgs/Pose2D)
- 레버암 보정: h(x) = [x, y]^T + R(psi)*lever
"""

import math
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
try:
    import tf2_ros
    import geometry_msgs.msg as geom_msgs
    HAS_TF = True
except Exception:
    HAS_TF = False


def ang_norm(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class GPSIMU_EKF:
    """
    동역학:
      x'   = x + v*dt*cos(psi)
      y'   = y + v*dt*sin(psi)
      psi' = psi + r*dt
      v'   = v
    측정:
      z_gps = [x, y]^T + R(psi)*lever
    """

    def __init__(self):
        # ===== 파라미터 =====
        self.frame_id       = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.lever_x = float(rospy.get_param("~lever_x", 0.20))
        self.lever_y = float(rospy.get_param("~lever_y", 0.15))

        gps_pos_std  = float(rospy.get_param("~gps_pos_std", 0.12))
        sig_x   = float(rospy.get_param("~sig_x", 0.20))
        sig_y   = float(rospy.get_param("~sig_y", 0.20))
        sig_psi = float(rospy.get_param("~sig_psi", 0.6*math.pi/180.0))
        sig_v   = float(rospy.get_param("~sig_v", 0.50))
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        self.timer_hz   = float(rospy.get_param("~timer_hz", 100.0))

        self.odom_topic     = rospy.get_param("~odom_topic", "odometry/ekf")
        self.gps_topic      = rospy.get_param("~gps_topic", "/gps/utm_pos1")
        self.imu_data_topic = rospy.get_param("~imu_data_topic", "/imu/data")

        # ===== 상태/공분산 =====
        self.x = np.zeros((4,1))  # [x,y,psi,v]^T
        self.P = np.diag([5.0, 5.0, (10.0*math.pi/180.0)**2, 4.0])

        self.Q_base = np.diag([sig_x**2, sig_y**2, sig_psi**2, sig_v**2])
        self.R_gps  = np.diag([gps_pos_std**2, gps_pos_std**2])

        # 입력/측정 버퍼
        self.r_yawrate = 0.0           # [rad/s]
        self.gps_meas  = None          # [[x],[y]]
        self.last_time = None

        # ===== ROS I/O =====
        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.sub_gps      = rospy.Subscriber(self.gps_topic, Pose2D, self.cb_gps, queue_size=10)
        self.sub_imu_data = rospy.Subscriber(self.imu_data_topic, Imu, self.cb_imu_data, queue_size=50)

        period = 1.0 / max(1.0, self.timer_hz)
        rospy.Timer(rospy.Duration(period), self.on_timer)

        if self.publish_tf and HAS_TF:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        else:
            self.tf_broadcaster = None

        rospy.loginfo("[gps_imu_ekf] started. (/imu/yaw 미사용)")

    # ================= 콜백 =================
    def cb_gps(self, msg: Pose2D):
        self.gps_meas = np.array([[msg.x],[msg.y]])
        self.update_gps()
        self.publish()

    def cb_imu_data(self, msg: Imu):
        self.r_yawrate = float(msg.angular_velocity.z)

    # ================= EKF 핵심 =================
    def predict(self, dt: float):
        if dt <= 0.0 or dt > 1.0:
            return
        x, y, psi, v = self.x.flatten()
        r = float(self.r_yawrate)

        x_p   = x   + v*dt*math.cos(psi)
        y_p   = y   + v*dt*math.sin(psi)
        psi_p = ang_norm(psi + r*dt)
        v_p   = v

        self.x = np.array([[x_p],[y_p],[psi_p],[v_p]])

        F = np.eye(4)
        F[0,2] = -v*dt*math.sin(psi)
        F[0,3] =  dt*math.cos(psi)
        F[1,2] =  v*dt*math.cos(psi)
        F[1,3] =  dt*math.sin(psi)

        Q = self.Q_base * dt
        self.P = F @ self.P @ F.T + Q

    def update_gps(self):
        if self.gps_meas is None:
            return
        lx, ly = self.lever_x, self.lever_y
        psi = self.x[2,0]
        c, s = math.cos(psi), math.sin(psi)

        h = np.array([
            [self.x[0,0] + c*lx - s*ly],
            [self.x[1,0] + s*lx + c*ly]
        ])
        z = self.gps_meas
        y = z - h

        H = np.zeros((2,4))
        H[0,0] = 1.0
        H[1,1] = 1.0
        H[0,2] = -s*lx - c*ly
        H[1,2] =  c*lx - s*ly

        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y
        self.x += dx
        self.x[2,0] = ang_norm(self.x[2,0])

        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    # ================= 루프/퍼블리시 =================
    def on_timer(self, _evt):
        t = rospy.Time.now().to_sec()
        if self.last_time is None:
            self.last_time = t
            return
        dt = t - self.last_time
        self.last_time = t

        self.predict(dt)
        self.publish()

    def publish(self):
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id  = self.child_frame_id

        odom.pose.pose.position.x = float(self.x[0,0])
        odom.pose.pose.position.y = float(self.x[1,0])
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, float(self.x[2,0]))
        odom.pose.pose.orientation = Quaternion(*q)

        cov = np.zeros((6,6))
        cov[0,0] = self.P[0,0]
        cov[1,1] = self.P[1,1]
        cov[5,5] = self.P[2,2]
        odom.pose.covariance = cov.flatten().tolist()

        odom.twist.twist.linear.x  = float(self.x[3,0])
        odom.twist.twist.angular.z = float(self.r_yawrate)
        self.pub_odom.publish(odom)

        if self.publish_tf and self.tf_broadcaster is not None:
            tmsg = geom_msgs.TransformStamped()
            tmsg.header.stamp = now
            tmsg.header.frame_id = self.frame_id
            tmsg.child_frame_id  = self.child_frame_id
            tmsg.transform.translation.x = float(self.x[0,0])
            tmsg.transform.translation.y = float(self.x[1,0])
            tmsg.transform.translation.z = 0.0
            tmsg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tmsg)


def main():
    rospy.init_node("gps_imu_ekf")
    GPSIMU_EKF()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
