#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import pandas as pd
import os
import csv
import re
from datetime import datetime

######### ros 통신을 위한 라이브러리 ###########
import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32, Int32, String  # 신호등 상태 수신용 String
from std_srvs.srv import Trigger  # ★ 카메라 노드 서비스 타입 호환 (Trigger)

# 전역 변수
current_x = 0.0
current_y = 0.0
current_yaw = 0.0

# 신호등 상태(카메라 노드에서 수신)
current_tl_state = "unknown"  # 'red','yellow','green','rl','unknown' 외 미지 문자열 가능

velocity = Twist()
wheelbase = 0.73
L = 3.0
kp_angular = 1.0

# GPS 안테나와 뒷바퀴 축 사이의 거리 (m)
REAR_AXLE_OFFSET = 0.73

# 사용자가 설정할 웨이포인트 인덱스 리스트
litte_L_point = [*range(454, 477), *range(905, 986)]  # 전방주시거리를 줄여서 접근해야할 웨이포인트
slow_point = [*range(60, 94),*range(107, 125),*range(136, 150),
              *range(160, 171),*range(183, 204),*range(266, 378),
              *range(430, 498), *range(506, 518),
              *range(543, 555), *range(575, 587), *range(678, 691),*range(734,753),
              *range(851,864),
              *range(906, 986),*range(1004, 1033)]   # 천천히 이동해야할 웨이포인트 굴절코스 *range(400, 625)    s자코스 *range(890,1122)
back_point = [*range(462, 468), *range(932, 958)]  # 후진으로 이동해야할 웨이포인트
wait_point = [467, 957]  # 평지 정지 웨이포인트 / 신호등 앞에서 + 후진 했을 때 멈추게 하면 됨

uphill_wait_point = [46]  # 오르막 정지 웨이포인트
uphill_drive_point = [*range(36,60), *range(770, 840)]  # 오르막에서 출력을 올릴 웨이포인트

t_parking_decision_point = [453]  # T자 주차 결정을 위해 정지할 웨이포인트
parallel_parking_decision_point = [905]  # 평행 주차 결정을 위해 정지할 웨이포인트
	
# 신호등 대기 웨이포인트
straight_traffic_list = [217,218, 411,412] # 직진 신호 대기 인덱스
left_traffic_list = [613,614]  # 좌회전 신호 대기 인덱스


# ─────────────────────────────────────────────────────
# 콜백/유틸
# ─────────────────────────────────────────────────────
def tl_state_callback(msg: String):
    """카메라 노드(/traffic_light_state)에서 오는 최종 색상 문자열 수신."""
    global current_tl_state
    current_tl_state = (msg.data or "").strip().lower()


def load_csv_xy(path):
    xs, ys = [], []
    with open(path, 'r') as f:
        rdr = csv.reader(f)
        rows = []
        first = next(rdr)
        try:
            float(first[0])
            rows.append(first)
        except:
            pass
        rows.extend(list(rdr))
    for r in rows:
        if len(r) < 2: continue
        xs.append(float(r[0]));
        ys.append(float(r[1]))
    return xs, ys


def gps_callback(msg):
    global current_x, current_y
    current_x = msg.x
    current_y = msg.y


def imu_callback(msg):
    global current_yaw
    current_yaw = math.radians(msg.data)


def getDistance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)


class Trajectory:
    def __init__(self, traj_x, traj_y):
        self.traj_x = traj_x
        self.traj_y = traj_y
        # 웨이포인트 설정 굴절코스 시작은 400   s자코스시작은 890
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


# ─────────────────────────────────────────────────────
# Trigger 서비스 호환용: message에서 code=1/2 → "A"/"B" 매핑
# (카메라 노드: /GetParkingPath, /GetParallelParkingPath)
#  - 성공 시 TriggerResponse.message 예: "confirmed t, ... code=2"
#  - 여기서는 임의로 1→"A", 2→"B"로 사용 (합의 필요)
# ─────────────────────────────────────────────────────
_code_re = re.compile(r"code\s*=\s*([0-9]+)")


def _extract_path_type_from_message(message: str, default="A"):
    if not message:
        return default
    m = _code_re.search(message.lower())
    if not m:
        return default
    code = m.group(1)
    if code == "1":
        return "A"
    if code == "2":
        return "B"
    return default


def main():
    rospy.init_node('gps_waypoint_controller')
    rate = rospy.Rate(10)

    # ─ 서비스 대기 (카메라 노드 Trigger 서비스와 호환)
    print("T자 주차 서비스를 기다리는 중... (/GetParkingPath, Trigger)")
    rospy.wait_for_service('/GetParkingPath')

    print("평행 주차 서비스를 기다리는 중... (/GetParallelParkingPath, Trigger)")
    rospy.wait_for_service('/GetParallelParkingPath')

    # ─ ServiceProxy 교체 (Trigger)
    get_t_parking_path_service = rospy.ServiceProxy('/GetParkingPath', Trigger)
    get_parallel_parking_path_service = rospy.ServiceProxy('/GetParallelParkingPath', Trigger)

    # ─ 로그 및 웨이포인트 로딩
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    log_dir = os.path.expanduser(f"/home/icas/{stamp}")
    os.makedirs(log_dir, exist_ok=True)
    log_rows = []

    # 웨이포인트 파일 위치
    waypoint_file = pd.read_csv('/home/icas/basic.csv')
    traj_x = waypoint_file['x'].tolist()
    traj_y = waypoint_file['y'].tolist()
    if len(traj_x) == 0:
        rospy.logerr("Waypoint list is empty.")
        return

    traj = Trajectory(traj_x, traj_y)

    waypoint_pub = rospy.Publisher('/current_waypoint', Int32, queue_size=1)
    velocity_pub = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/gps/utm_pos1', Pose2D, gps_callback)
    rospy.Subscriber('/imu/yaw', Float32, imu_callback)

    # ★ 카메라 노드 신호등 상태 구독 (색상 문자열)
    rospy.Subscriber('/traffic_light_state', String, tl_state_callback)

    rospy.sleep(1)

    # --- wait_point 로직을 위한 상태 변수
    is_waiting_now = False  # 평지 정지
    is_uphill_waiting_now = False  # 오르막 정지
    is_t_parking_waiting = False
    is_parallel_parking_waiting = False

    # ★ 신호등 대기 상태
    is_straight_waiting = False
    is_left_waiting = False

    wait_start_time = None
    WAIT_DURATION = 3
    STOP_DISTANCE_THRESHOLD = 2
    triggered_wait_idx = None

    # ★ 신호등 정지 타임아웃(30초) 처리용
    TL_STOP_HOLD_TIMEOUT = 30.0
    tl_stop_since = None
    tl_force_go_active = False  # 30초 초과 시 강제 주행

    # 인식 가능한 기본 클래스 집합
    TL_KNOWN = {'red', 'yellow', 'green', 'rl', 'unknown'}

    def normalize_tl(s: str) -> str:
        s = (s or "").strip().lower()
        return s if s in TL_KNOWN else s  # 모르는 문자열은 그대로 두되, 판정은 '주행'

    while not rospy.is_shutdown():
        # 후진 모드 여부에 따라 제어 기준 좌표 설정
        if traj.last_idx in back_point:
            control_x = current_x - REAR_AXLE_OFFSET * math.cos(current_yaw)
            control_y = current_y - REAR_AXLE_OFFSET * math.sin(current_yaw)
        else:
            control_x = current_x
            control_y = current_y

        # 동적 L 설정
        current_L = 1.0 if (traj.last_idx in litte_L_point) else L

        target_point = traj.getTargetPoint([control_x, control_y], current_L)

        mode = "fwd"
        steering_command = 0.0

        # ===================================================================
        # 모든 정지 상태 '진입' 로직
        # ===================================================================
        if (not is_uphill_waiting_now and not is_waiting_now and
                not is_t_parking_waiting and not is_parallel_parking_waiting and
                not is_straight_waiting and not is_left_waiting):

            # 오르막 정지
            for wait_idx in uphill_wait_point:
                wait_wp_pos = traj.getPoint(wait_idx)
                if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                    print(f"오르막 정지 지점 근접. {WAIT_DURATION}초 동안 위치를 유지합니다.")
                    wait_start_time = rospy.get_time()
                    is_uphill_waiting_now = True
                    triggered_wait_idx = wait_idx
                    break

            # 평지 정지
            if not is_uphill_waiting_now:
                for wait_idx in wait_point:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        print(f"평지 정지 지점 근접. {WAIT_DURATION}초 동안 정지합니다.")
                        wait_start_time = rospy.get_time()
                        is_waiting_now = True
                        triggered_wait_idx = wait_idx
                        break

            # T자 주차
            if (not is_uphill_waiting_now) and (not is_waiting_now):
                for wait_idx in t_parking_decision_point:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        print(f"T자 주차 지점 근접. {WAIT_DURATION}초 동안 정지합니다.")
                        wait_start_time = rospy.get_time()
                        is_t_parking_waiting = True
                        triggered_wait_idx = wait_idx
                        break

            # 평행 주차
            if (not is_uphill_waiting_now) and (not is_waiting_now) and (not is_t_parking_waiting):
                for wait_idx in parallel_parking_decision_point:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        print(f"평행 주차 지점 근접. {WAIT_DURATION}초 동안 정지합니다.")
                        wait_start_time = rospy.get_time()
                        is_parallel_parking_waiting = True
                        triggered_wait_idx = wait_idx
                        break

            # 직진 신호 대기 진입
            if (not is_uphill_waiting_now) and (not is_waiting_now) and (not is_t_parking_waiting) and (
            not is_parallel_parking_waiting):
                for wait_idx in straight_traffic_list:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        print(f"직진 신호등 대기 지점 근접. 신호에 따라 대기/주행 결정합니다.")
                        is_straight_waiting = True
                        triggered_wait_idx = wait_idx
                        tl_stop_since = None
                        tl_force_go_active = False
                        break

            # 좌회전 신호 대기 진입
            if (not is_uphill_waiting_now) and (not is_waiting_now) and (not is_t_parking_waiting) and (
            not is_parallel_parking_waiting) and (not is_straight_waiting):
                for wait_idx in left_traffic_list:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        print(f"좌회전 신호등 대기 지점 근접. 신호에 따라 대기/주행 결정합니다.")
                        is_left_waiting = True
                        triggered_wait_idx = wait_idx
                        tl_stop_since = None
                        tl_force_go_active = False
                        break

        # ===================================================================
        # 모든 정지 상태 '수행' 로직
        # ===================================================================

        # 오르막 정지
        if is_uphill_waiting_now:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < WAIT_DURATION:

                velocity.linear.x = 30

                alpha = math.atan2(
                    (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                        current_yaw),
                    (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                        current_yaw)
                )
                steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
                steering_command = steering_angle * kp_angular

                velocity.angular.z = steering_command

                mode = "uphill_wait"
                print(f"오르막 정지 유지 중... ({elapsed_time:.1f}/{WAIT_DURATION:.1f} 초)")
            else:
                print("오르막 정지 완료. 주행을 재개합니다.")
                if triggered_wait_idx in uphill_wait_point:
                    uphill_wait_point.remove(triggered_wait_idx)
                is_uphill_waiting_now = False
                wait_start_time = None
                triggered_wait_idx = None
                mode = "fwd"

        # 평지 정지
        elif is_waiting_now:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < WAIT_DURATION:
                velocity.linear.x = 0.0

                alpha = math.atan2(
                    (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                        current_yaw),
                    (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                        current_yaw)
                )
                steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
                steering_command = steering_angle * kp_angular

                velocity.angular.z = steering_command

                mode = "wait"
                print(f"평지 정지 중... ({elapsed_time:.1f}/{WAIT_DURATION:.1f} 초)")
            else:
                print("평지 정지 완료. 주행을 재개합니다.")
                if triggered_wait_idx in wait_point:
                    wait_point.remove(triggered_wait_idx)
                is_waiting_now = False
                wait_start_time = None
                triggered_wait_idx = None
                mode = "fwd"

        # T자 주차 정지
        elif is_t_parking_waiting:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < 2:
                velocity.linear.x = 0.0
                alpha = math.atan2(
                    (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                        current_yaw),
                    (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                        current_yaw)
                )
                steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
                steering_command = steering_angle * kp_angular

                velocity.angular.z = steering_command

                mode = "T_wait"
                print(f"T 주차위치 파악중... 2초)")
            else:
                try:
                    print("카메라 노드에 T자 주차 경로 서비스를 요청합니다... (/GetParkingPath)")
                    resp = get_t_parking_path_service()
                    if not resp.success:
                        rospy.logwarn("T 주차 서비스 실패: %s", resp.message)
                        path_type = None
                    else:
                        path_type = _extract_path_type_from_message(resp.message, default="A")
                    print(f"경로 응답 수신: path_type='{path_type}'")
                    parking_csv_path = None
                    if path_type == "A":
                        parking_csv_path = '/home/icas/T_parking_A.csv'
                    elif path_type == "B":
                        parking_csv_path = '/home/icas/T_parking_B.csv'

                    if parking_csv_path:
                        new_parking_x, new_parking_y = load_csv_xy(parking_csv_path)
                        T_PARKING_START_IDX = 454
                        wp_count_to_replace = len(new_parking_x)
                        print(f"{T_PARKING_START_IDX}번 인덱스부터 {wp_count_to_replace}개의 웨이포인트를 교체합니다.")
                        for i in range(wp_count_to_replace):
                            idx_to_replace = T_PARKING_START_IDX + i
                            if idx_to_replace < len(traj.traj_x):
                                traj.traj_x[idx_to_replace] = new_parking_x[i]
                                traj.traj_y[idx_to_replace] = new_parking_y[i]
                except rospy.ServiceException as e:
                    print(f"서비스 호출에 실패했습니다: {e}")
                print("T 주차 경로 파악 완료. 주행을 재개합니다.")
                if triggered_wait_idx in t_parking_decision_point:
                    t_parking_decision_point.remove(triggered_wait_idx)
                is_t_parking_waiting = False
                wait_start_time = None
                triggered_wait_idx = None
                mode = "fwd"

        # 평행 주차 정지
        elif is_parallel_parking_waiting:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < 2:
                velocity.linear.x = 0.0
                alpha = math.atan2(
                    (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                        current_yaw),
                    (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                        current_yaw)
                )
                steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
                steering_command = steering_angle * kp_angular

                velocity.angular.z = steering_command

                mode = "parallel_wait"
                print(f"평행 주차위치 파악중... 2초)")
            else:
                try:
                    print("카메라 노드에 평행 주차 경로 서비스를 요청합니다... (/GetParallelParkingPath)")
                    resp = get_parallel_parking_path_service()
                    if not resp.success:
                        rospy.logwarn("평행 주차 서비스 실패: %s", resp.message)
                        path_type = None
                    else:
                        path_type = _extract_path_type_from_message(resp.message, default="A")
                    print(f"경로 응답 수신: path_type='{path_type}'")
                    parking_csv_path = None
                    if path_type == "A":
                        parking_csv_path = '/home/icas/p_parking_B.csv'
                    elif path_type == "B":
                        parking_csv_path = '/home/icas/p_parking_A.csv'

                    if parking_csv_path:
                        new_parking_x, new_parking_y = load_csv_xy(parking_csv_path)
                        PARALLEL_PARKING_START_IDX = 906
                        wp_count_to_replace = len(new_parking_x)
                        print(f"{PARALLEL_PARKING_START_IDX}번 인덱스부터 {wp_count_to_replace}개의 웨이포인트를 교체합니다.")
                        for i in range(wp_count_to_replace):
                            idx_to_replace = PARALLEL_PARKING_START_IDX + i
                            if idx_to_replace < len(traj.traj_x):
                                traj.traj_x[idx_to_replace] = new_parking_x[i]
                                traj.traj_y[idx_to_replace] = new_parking_y[i]
                except rospy.ServiceException as e:
                    print(f"서비스 호출에 실패했습니다: {e}")
                print("평행 주차 경로 파악 완료. 주행을 재개합니다.")
                if triggered_wait_idx in parallel_parking_decision_point:
                    parallel_parking_decision_point.remove(triggered_wait_idx)
                is_parallel_parking_waiting = False
                wait_start_time = None
                triggered_wait_idx = None
                mode = "fwd"

        # ★ 직진 신호 대기 처리
        elif is_straight_waiting:
            # 규칙: 직진에서는 red, yellow 정지 / green, unknown 주행
            tl = normalize_tl(current_tl_state)
            base_stop = (tl in ('red', 'yellow'))
            # 미지의 문자열(known set 밖) → 주행
            if tl not in {'red', 'yellow', 'green', 'rl', 'unknown'}:
                base_stop = False

            nowt = rospy.get_time()
            if base_stop and not tl_force_go_active:
                if tl_stop_since is None:
                    tl_stop_since = nowt
                elif (nowt - tl_stop_since) >= TL_STOP_HOLD_TIMEOUT:
                    print("[직진] 신호 대기 30초 초과 → 강제 주행 전환")
                    tl_force_go_active = True
            else:
                tl_stop_since = None  # 멈출 필요 없거나, 신호 바뀌면 타이머 리셋

            if base_stop and not tl_force_go_active:
                velocity.linear.x = 0.0
                alpha = math.atan2(
                    (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                        current_yaw),
                    (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                        current_yaw)
                )
                steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
                steering_command = steering_angle * kp_angular

                velocity.angular.z = steering_command

                mode = "straight_wait_stop"
                held = 0.0 if tl_stop_since is None else (nowt - tl_stop_since)
                print(f"[직진] 신호등={tl} → 정지 유지 ({held:.1f}s)")
            else:
                print(f"[직진] 신호등={tl}{' (강제주행)' if tl_force_go_active else ''} → 주행")
                if triggered_wait_idx in straight_traffic_list:
                    straight_traffic_list.remove(triggered_wait_idx)
                is_straight_waiting = False
                tl_stop_since = None
                tl_force_go_active = False
                mode = "fwd"

        # ★ 좌회전 신호 대기 처리
        elif is_left_waiting:
            # 규칙: 좌회전에서는 red, yellow, green 정지 / rl, unknown 주행
            tl = normalize_tl(current_tl_state)
            base_stop = (tl in ('red', 'yellow', 'green'))
            if tl not in {'red', 'yellow', 'green', 'rl', 'unknown'}:
                base_stop = False  # 미지의 문자열은 주행

            nowt = rospy.get_time()
            if base_stop and not tl_force_go_active:
                if tl_stop_since is None:
                    tl_stop_since = nowt
                elif (nowt - tl_stop_since) >= TL_STOP_HOLD_TIMEOUT:
                    print("[좌회전] 신호 대기 30초 초과 → 강제 주행 전환")
                    tl_force_go_active = True
            else:
                tl_stop_since = None

            if base_stop and not tl_force_go_active:
                velocity.linear.x = 0.0
                alpha = math.atan2(
                    (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                        current_yaw),
                    (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                        current_yaw)
                )
                steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
                steering_command = steering_angle * kp_angular

                velocity.angular.z = steering_command

                mode = "left_wait_stop"
                held = 0.0 if tl_stop_since is None else (nowt - tl_stop_since)
                print(f"[좌회전] 신호등={tl} → 정지 유지 ({held:.1f}s)")
            else:
                print(f"[좌회전] 신호등={tl}{' (강제주행)' if tl_force_go_active else ''} → 주행")
                if triggered_wait_idx in left_traffic_list:
                    left_traffic_list.remove(triggered_wait_idx)
                is_left_waiting = False
                tl_stop_since = None
                tl_force_go_active = False
                mode = "fwd"

        # 후진
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
            # 후진할 때 속도
            velocity.linear.x = -15.0
            velocity.angular.z = steering_command

        # 오르막 주행 가속
        elif traj.last_idx in uphill_drive_point:
            mode = "uphill_drive"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                    current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                    current_yaw)
            )
            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            # 오르막과 가속구간 속도
            velocity.linear.x = 56
            velocity.angular.z = steering_command

        # 슬로우
        elif traj.last_idx in slow_point:
            mode = "slow"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                    current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                    current_yaw)
            )
            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            # 슬로우일 때 속도
            velocity.linear.x = 20
            velocity.angular.z = steering_command

        # 일반 전진
        else:
            mode = "fwd"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(
                    current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(
                    current_yaw)
            )
            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            # 기본 속도
            velocity.linear.x = 32
            velocity.angular.z = steering_command

        # 로그 및 퍼블리시
        print(f"현재 추종 웨이포인트: {traj.last_idx}, Mode: {mode}, Look-ahead(L): {current_L}")
        print(f"현재속도: {velocity.linear.x:.2f}, 현재 조향각: {velocity.angular.z:.2f}")
        print(f"목표점까지의 거리: {getDistance([control_x, control_y], target_point):.2f}\n")

        log_rows.append({
            "time": rospy.get_time(),
            "gps_x": current_x, "gps_y": current_y,
            "control_x": control_x, "control_y": control_y,
            "yaw": current_yaw, "wp_idx": traj.last_idx, "mode": mode,
            "v_cmd": float(velocity.linear.x), "steer_cmd": float(velocity.angular.z),
            "tl_state": current_tl_state
        })

        waypoint_pub.publish(traj.last_idx)
        velocity_pub.publish(velocity)
        rate.sleep()

    # 종료
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
