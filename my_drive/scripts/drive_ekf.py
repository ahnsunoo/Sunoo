#!/usr/bin/env python3

import math
import pandas as pd
import os
from datetime import datetime

# --- EKF 연동을 위한 라이브러리 ---
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# 전역 변수
current_x = 0.0
current_y = 0.0
current_yaw = 0.0 # 라디안(radian) 단위

velocity = Twist()
wheelbase = 0.73
L = 3.0
kp_angular = 1.0
REAR_AXLE_OFFSET = 0.73

# 웨이포인트 인덱스 리스트
litte_L_point = [516, 517, 518, 519, 520, 521, 522] 
slow_point = [516, 517, 518, 519, 520, 521] 
back_point = [518, 519] 
wait_point = [519]
uphill_wait_point = [] 
uphill_drive_point = []

def odometry_callback(msg):
    """
    EKF로부터 융합된 위치/자세 정보를 받아 전역 변수를 업데이트합니다.
    """
    global current_x, current_y, current_yaw
    
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    current_yaw = yaw

def getDistance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

class Trajectory:
    def __init__(self, traj_x, traj_y):
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0
    def getPoint(self, idx):
        return [self.traj_x[idx], self.traj_y[idx]]
    def getTargetPoint(self, pos, look_ahead_dist):
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)
        while curr_dist < look_ahead_dist and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)
        self.last_idx = target_idx
        return self.getPoint(target_idx)


def main():
    rospy.init_node('gps_waypoint_controller_ekf')
    rate = rospy.Rate(10)

    # 로그 및 웨이포인트 로딩
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    log_dir = os.path.expanduser(f"~/{stamp}")
    os.makedirs(log_dir, exist_ok=True)
    log_rows = []
    
    csv_path = os.path.expanduser("~/sibal4.csv") 
    waypoint_file = pd.read_csv(csv_path)
    traj_x = waypoint_file['x'].tolist()
    traj_y = waypoint_file['y'].tolist()
    traj = Trajectory(traj_x, traj_y)

    velocity_pub = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)
    rospy.sleep(1)

    # 정지 로직 상태 변수
    is_waiting_now = False        
    is_uphill_waiting_now = False 
    wait_start_time = None
    WAIT_DURATION = 3
    STOP_DISTANCE_THRESHOLD = 1
    triggered_wait_idx = None
    
    while not rospy.is_shutdown():
        
        # target_point를 항상 먼저 계산
        if traj.last_idx in back_point:
            control_x = current_x - REAR_AXLE_OFFSET * math.cos(current_yaw)
            control_y = current_y - REAR_AXLE_OFFSET * math.sin(current_yaw)
        else:
            control_x = current_x
            control_y = current_y

        if traj.last_idx in litte_L_point:
            current_L = 1.0
        else:
            current_L = L

        target_point = traj.getTargetPoint([control_x, control_y], current_L)

        mode = "fwd"
        steering_command = 0.0

        # 정지 지점과의 거리 체크
        if not is_uphill_waiting_now and not is_waiting_now:
            for wait_idx in uphill_wait_point:
                wait_wp_pos = traj.getPoint(wait_idx)
                if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                    rospy.loginfo(f"Uphill wait point {wait_idx} approaching...")
                    wait_start_time = rospy.get_time()
                    is_uphill_waiting_now = True
                    triggered_wait_idx = wait_idx
                    break

        if not is_uphill_waiting_now and not is_waiting_now:
            for wait_idx in wait_point:
                wait_wp_pos = traj.getPoint(wait_idx)
                if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                    rospy.loginfo(f"Wait point {wait_idx} approaching...")
                    wait_start_time = rospy.get_time()
                    is_waiting_now = True
                    triggered_wait_idx = wait_idx
                    break

        # 정지 상태일 경우의 로직
        if is_uphill_waiting_now:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < WAIT_DURATION:
                mode = "uphill_wait"
                velocity.linear.x = 30
                steering_command = 0.0
            else:
                uphill_wait_point.remove(triggered_wait_idx)
                is_uphill_waiting_now = False
                mode = "fwd"

        elif is_waiting_now:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < WAIT_DURATION:
                mode = "wait"
                velocity.linear.x = 0.0
                steering_command = 0.0
            else:
                wait_point.remove(triggered_wait_idx)
                is_waiting_now = False
                mode = "fwd"
        
        # 주행 로직
        elif traj.last_idx in back_point:
            mode = "back"
            target_angle = math.degrees(math.atan2(
                target_point[1] - control_y,
                target_point[0] - control_x
            ))
            current_angle_deg = math.degrees(current_yaw)
            current_angle_deg = (current_angle_deg + 180) % 360
            diff_angle = target_angle - current_angle_deg
            steering_angle = ((diff_angle + 180) % 360) - 180
            steering_angle = (-1) * steering_angle
            steering_command = steering_angle * kp_angular
            velocity.linear.x = -20.0
            velocity.angular.z = steering_command

        elif traj.last_idx in uphill_drive_point:
            mode = "uphill_drive"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(current_yaw)
            )
            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            velocity.linear.x = 50
            velocity.angular.z = steering_command

        elif traj.last_idx in slow_point:
            mode = "slow"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(current_yaw)
            )
            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            velocity.linear.x = 20
            velocity.angular.z = steering_command

        else:  # 일반 전진 주행
            mode = "fwd"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(current_yaw)
            )
            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            velocity.linear.x = 27
            velocity.angular.z = steering_command

        # [수정] 요청하신 형식의 print문 추가
        print(f"현재 추종 웨이포인트: {traj.last_idx}, Mode: {mode}, Look-ahead(L): {current_L}")
        print(f"현재속도: {velocity.linear.x:.2f}, 현재 조향각: {velocity.angular.z:.2f}")
        print(f"목표점까지의 거리: {getDistance([control_x, control_y], target_point):.2f}\n")

        # 로그 기록 및 퍼블리시
        log_rows.append({
            "time": rospy.get_time(),
            "gps_x": current_x, "gps_y": current_y,
            "control_x": control_x, "control_y": control_y,
            "yaw": current_yaw, "wp_idx": traj.last_idx, "mode": mode,
            "v_cmd": float(velocity.linear.x), "steer_cmd": float(velocity.angular.z)
        })

        velocity_pub.publish(velocity)
        rate.sleep()

    # 프로그램 종료
    velocity.linear.x = 0
    velocity.angular.z = 0
    velocity_pub.publish(velocity)
    print("Waypoint driving finished.")

    df_log = pd.DataFrame(log_rows)
    out_csv = os.path.join(log_dir, "track.csv")
    df_log.to_csv(out_csv, index=False, encoding="utf-8")
    print(f"[LOG] Saved tracking data to: {out_csv}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
