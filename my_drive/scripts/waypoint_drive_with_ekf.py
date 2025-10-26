#!/usr/bin/env python3

import math
import pandas as pd
import os
from datetime import datetime

#########ros통신을 위한 라이브러리 추가###########
import rospy
from geometry_msgs.msg import Pose2D, Twist
# ★ EKF 오도메트리/쿼터니언→euler 변환 추가
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# /imu/yaw(Float32)는 더 이상 안 씀
# from std_msgs.msg import Float32

# 전역 변수
current_x = 0.0
current_y = 0.0
current_yaw = 0.0   # rad

velocity = Twist()
wheelbase = 0.73
L = 3.0
kp_angular = 1.0

# GPS 안테나와 뒷바퀴 축 사이의 거리 (m)
REAR_AXLE_OFFSET = 0.73

# 사용자가 설정할 웨이포인트 인덱스 리스트
litte_L_point = [516,517,518,519,520,521,522] #전방주시거리를 줄여서 접근해야할 웨이포인트
slow_point = [516,517,518,519,520,521]        #천천히 이동해야할 웨이포인트
back_point = [518,519]                        #후진으로 이동해야할 웨이포인트
wait_point = [519]                            #평지 정지 웨이포인트
uphill_wait_point = []                        # 오르막 정지 웨이포인트

# ★ EKF 오도메트리 콜백: x,y,yaw(rad) 직접 반영
def ekf_odom_callback(msg: Odometry):
    global current_x, current_y, current_yaw
    # 위치
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    # yaw(rad)
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    current_yaw = yaw  # rad (기존 로직은 rad 기준으로 동작)

# 기존 콜백은 더 이상 사용하지 않음(참고용으로 남겨둠)
# def gps_callback(msg: Pose2D):
#     global current_x, current_y
#     current_x = msg.x
#     current_y = msg.y
#
# def imu_callback(msg: Float32):
#     global current_yaw
#     current_yaw = math.radians(msg.data)

def getDistance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

class Trajectory:
    # ... (Trajectory 클래스는 변경 없음) ...
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
    rospy.init_node('gps_waypoint_controller')
    rate = rospy.Rate(10)

    # ... (로그 및 웨이포인트 로딩 코드는 변경 없음) ...
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    log_dir = os.path.expanduser(f"/home/icas/{stamp}")
    os.makedirs(log_dir, exist_ok=True)
    log_rows = []

    waypoint_file = pd.read_csv('/home/icas/sibal4.csv')
    traj_x = waypoint_file['x'].tolist()
    traj_y = waypoint_file['y'].tolist()
    if len(traj_x) == 0:
        rospy.logerr("Waypoint list is empty.")
        return

    traj = Trajectory(traj_x, traj_y)

    velocity_pub = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)

    # ★ 구독 토픽 변경: EKF 오도메트리만 구독
    rospy.Subscriber('/odometry/ekf', Odometry, ekf_odom_callback)
    # (이전) 원시 GPS/IMU는 더 이상 직접 사용하지 않음
    # rospy.Subscriber('/gps/utm_pos1', Pose2D, gps_callback)
    # rospy.Subscriber('/imu/yaw', Float32, imu_callback)

    rospy.sleep(1)

    # --- [추가됨] --- wait_point 로직을 위한 상태 변수 (원본 유지)
    is_waiting_now = False
    is_uphill_waiting_now = False
    wait_start_time = None
    WAIT_DURATION = 3
    STOP_DISTANCE_THRESHOLD = 1
    triggered_wait_idx = None

    while not rospy.is_shutdown():
        # ======== 여기부터는 “주행 로직” 원본 그대로 유지 ========
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

        # ========= 정지 로직 (원본 유지) =========
        if not is_uphill_waiting_now and not is_waiting_now:
            for wait_idx in uphill_wait_point:
                wait_wp_pos = traj.getPoint(wait_idx)
                distance_to_wait_point = getDistance([control_x, control_y], wait_wp_pos)
                if distance_to_wait_point < STOP_DISTANCE_THRESHOLD:
                    print(f"오르막 정지 지점 근접. {WAIT_DURATION}초 동안 위치를 유지합니다.")
                    wait_start_time = rospy.get_time()
                    is_uphill_waiting_now = True
                    triggered_wait_idx = wait_idx
                    break

        if not is_uphill_waiting_now and not is_waiting_now:
            for wait_idx in wait_point:
                wait_wp_pos = traj.getPoint(wait_idx)
                distance_to_wait_point = getDistance([control_x, control_y], wait_wp_pos)
                if distance_to_wait_point < STOP_DISTANCE_THRESHOLD:
                    print(f"평지 정지 지점 근접. {WAIT_DURATION}초 동안 정지합니다.")
                    wait_start_time = rospy.get_time()
                    is_waiting_now = True
                    triggered_wait_idx = wait_idx
                    break

        if is_uphill_waiting_now:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < WAIT_DURATION:
                velocity.linear.x = 30
                steering_command = 0.0
                mode = "uphill_wait"
                print(f"오르막 정지 유지 중... ({elapsed_time:.1f}/{WAIT_DURATION:.1f} 초)")
            else:
                print("오르막 정지 완료. 주행을 재개합니다.")
                uphill_wait_point.remove(triggered_wait_idx)
                is_uphill_waiting_now = False
                wait_start_time = None
                triggered_wait_idx = None
                mode = "fwd"

        elif is_waiting_now:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < WAIT_DURATION:
                velocity.linear.x = 0.0
                steering_command = 0.0
                mode = "wait"
                print(f"평지 정지 중... ({elapsed_time:.1f}/{WAIT_DURATION:.1f} 초)")
            else:
                print("평지 정지 완료. 주행을 재개합니다.")
                wait_point.remove(triggered_wait_idx)
                is_waiting_now = False
                wait_start_time = None
                triggered_wait_idx = None
                mode = "fwd"

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
            steering_angle = (-1)*steering_angle
            steering_command = steering_angle * kp_angular
            velocity.linear.x = -20.0
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

        # 로그 기록 및 퍼블리시(원본 유지)
        print(f"현재 추종 웨이포인트: {traj.last_idx}, Mode: {mode}, Look-ahead(L): {current_L}")
        print(f"현재속도: {velocity.linear.x:.2f}, 현재 조향각: {velocity.angular.z:.2f}")
        print(f"목표점까지의 거리: {getDistance([control_x, control_y], target_point):.2f}\n")

        log_rows.append({
            "time": rospy.get_time(),
            "gps_x": current_x, "gps_y": current_y,   # 이제 EKF 위치가 기록됨
            "control_x": control_x, "control_y": control_y,
            "yaw": current_yaw, "wp_idx": traj.last_idx, "mode": mode,
            "v_cmd": float(velocity.linear.x), "steer_cmd": float(velocity.angular.z)
        })

        velocity_pub.publish(velocity)
        rate.sleep()

    # 종료 처리(원본 유지)
    velocity.linear.x = 0
    velocity.angular.z = 0
    velocity_pub.publish(velocity)
    print("it's over~")

    df_log = pd.DataFrame(log_rows)
    out_csv = os.path.join(log_dir, "track.csv")
    df_log.to_csv(out_csv, index=False, encoding="utf-8")
    print(f"[LOG] 주행 궤적 저장 완료: {out_csv}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
