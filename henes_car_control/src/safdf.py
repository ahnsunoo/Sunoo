#!/usr/bin/env python3

import math
import pandas as pd
import os
import csv
from datetime import datetime

######### ROS 서비스 타입/이름: 카메라(신호등+라바콘) 노드에 맞춰 변경 ###########
# - /GetParkingPath, /GetParallelParkingPath
# - std_srvs/Trigger 사용 (A/B 같은 path_type은 제공하지 않음)
from std_srvs.srv import Trigger

######### ROS 통신 ###########
import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32, Int32, String  # ← 신호등 상태 구독을 위해 String 추가

# 전역 변수
current_x = 0.0
current_y = 0.0
current_yaw = 0.0

# 신호등 상태 (카메라 노드 퍼블리시: /traffic_light_state, String: "red|yellow|rl|green|unknown")
tl_state = "unknown"

velocity = Twist()
wheelbase = 0.73
L = 3.0
kp_angular = 1.0

# GPS 안테나와 뒷바퀴 축 사이의 거리 (m)
REAR_AXLE_OFFSET = 0.73

# 사용자가 설정할 웨이포인트 인덱스 리스트
litte_L_point = [] #전방주시거리를 줄여서 접근해야할 웨이포인트
slow_point = [] #천천히 이동해야할 웨이포인트
back_point = [] #후진으로 이동해야할 웨이포인트
wait_point = [] #평지 정지 웨이포인트 / 신호등 앞에서 + 후진 했을 때 멈추게 하면 됨

uphill_wait_point = [] # 오르막 정지 웨이포인트
uphill_drive_point = [] # 오르막에서 출력을 올릴 웨이포인트

t_parking_decision_point = []      # T자 주차 결정을 위해 정지할 웨이포인트
parallel_parking_decision_point = [] # 평행 주차 결정을 위해 정지할 웨이포인트

# 신호등 지점 (직진/좌회전) — 인덱스 리스트
straight_traffic_list = []
left_traffic_list = []


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
        xs.append(float(r[0]))
        ys.append(float(r[1]))
    return xs, ys


def gps_callback(msg):
    global current_x, current_y
    current_x = msg.x
    current_y = msg.y


def imu_callback(msg):
    global current_yaw
    current_yaw = math.radians(msg.data)


def tl_callback(msg: String):
    """카메라 노드에서 주는 신호등 상태 수신 ('red','yellow','rl','green','unknown')"""
    global tl_state
    tl_state = (msg.data or "").strip().lower()


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

    # ── 라바콘 서비스: 카메라 노드와 동일한 이름/타입 사용
    print("T자 주차 서비스를 기다리는 중...")
    rospy.wait_for_service('/GetParkingPath')
    print("평행 주차 서비스를 기다리는 중...")
    rospy.wait_for_service('/GetParallelParkingPath')

    get_t_parking_path_service = rospy.ServiceProxy('/GetParkingPath', Trigger)
    get_parallel_parking_path_service = rospy.ServiceProxy('/GetParallelParkingPath', Trigger)

    # 로그/웨이포인트 로딩
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

    waypoint_pub = rospy.Publisher('/current_waypoint', Int32, queue_size=1)
    velocity_pub = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/gps/utm_pos1', Pose2D, gps_callback)
    rospy.Subscriber('/imu/yaw', Float32, imu_callback)

    # ── 신호등 상태 구독 (카메라 노드 퍼블리시)
    rospy.Subscriber('/traffic_light_state', String, tl_callback)

    rospy.sleep(1)

    # wait_point 등 상태 변수
    is_waiting_now = False
    is_uphill_waiting_now = False
    is_t_parking_waiting = False
    is_parallel_parking_waiting = False

    # 신호등 대기 상태 (직진/좌회전)
    is_straight_waiting = False
    is_left_waiting = False

    wait_start_time = None
    WAIT_DURATION = 3
    STOP_DISTANCE_THRESHOLD = 1
    triggered_wait_idx = None

    while not rospy.is_shutdown():
        # 후진 기준 좌표
        if traj.last_idx in back_point:
            control_x = current_x - REAR_AXLE_OFFSET * math.cos(current_yaw)
            control_y = current_y - REAR_AXLE_OFFSET * math.sin(current_yaw)
        else:
            control_x = current_x
            control_y = current_y

        # L 값
        if traj.last_idx in litte_L_point:
            current_L = 1.0
        else:
            current_L = L

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

            # T자 주차 정지
            if not is_uphill_waiting_now and not is_waiting_now:
                for wait_idx in t_parking_decision_point:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        print(f"T자 주차 지점 근접. {WAIT_DURATION}초 동안 정지합니다.")
                        wait_start_time = rospy.get_time()
                        is_t_parking_waiting = True
                        triggered_wait_idx = wait_idx
                        break

            # 평행 주차 정지
            if (not is_uphill_waiting_now and not is_waiting_now and
                not is_t_parking_waiting):
                for wait_idx in parallel_parking_decision_point:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        print(f"평행 주차 지점 근접. {WAIT_DURATION}초 동안 정지합니다.")
                        wait_start_time = rospy.get_time()
                        is_parallel_parking_waiting = True
                        triggered_wait_idx = wait_idx
                        break

            # ── 신호등(직진) : 빨간불일 때만 정지
            if (not is_uphill_waiting_now and not is_waiting_now and
                not is_t_parking_waiting and not is_parallel_parking_waiting and
                not is_straight_waiting and not is_left_waiting):
                for wait_idx in straight_traffic_list:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        if tl_state == "red":
                            print("직진 신호등 구간: 빨간불 → 정지 대기 진입")
                            is_straight_waiting = True
                            triggered_wait_idx = wait_idx
                        else:
                            # red가 아니면 통과(정지 진입 안함)
                            pass
                        break

            # ── 신호등(좌회전) : 빨간불이면 정지, 'rl'이면 주행
            if (not is_uphill_waiting_now and not is_waiting_now and
                not is_t_parking_waiting and not is_parallel_parking_waiting and
                not is_straight_waiting and not is_left_waiting):
                for wait_idx in left_traffic_list:
                    wait_wp_pos = traj.getPoint(wait_idx)
                    if getDistance([control_x, control_y], wait_wp_pos) < STOP_DISTANCE_THRESHOLD:
                        if tl_state == "red":
                            print("좌회전 신호등 구간: 빨간불 → 정지 대기 진입")
                            is_left_waiting = True
                            triggered_wait_idx = wait_idx
                        else:
                            # tl_state == 'rl' 포함: 정지 진입 안함 (그대로 주행)
                            pass
                        break

        # ===================================================================
        # 모든 정지 상태 '수행' 로직
        # ===================================================================

        # 오르막 정지
        if is_uphill_waiting_now:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < WAIT_DURATION:
                velocity.linear.x = 30
                steering_command = 0.0
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
                steering_command = 0.0
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

        # T자 주차 정지 → 서비스 호출(Trigger)
        elif is_t_parking_waiting:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < 2:
                velocity.linear.x = 0.0
                steering_command = 0.0
                mode = "T_wait"
                print("T 주차위치 파악중... 2초)")
            else:
                try:
                    print("카메라 노드에 T자 주차 경로 서비스를 요청합니다...")
                    resp = get_t_parking_path_service()
                    if not resp.success:
                        print(f"서비스 실패: {resp.message}")
                    else:
                        print(f"서비스 성공: {resp.message}")
                        # 카메라 노드는 A/B를 주지 않으므로 기본 CSV 한 개 사용
                        parking_csv_path = '/home/icas/T_parking_typeA.csv'
                        new_parking_x, new_parking_y = load_csv_xy(parking_csv_path)
                        T_PARKING_START_IDX = 501
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

        # 평행 주차 정지 → 서비스 호출(Trigger)
        elif is_parallel_parking_waiting:
            elapsed_time = rospy.get_time() - wait_start_time
            if elapsed_time < 2:
                velocity.linear.x = 0.0
                steering_command = 0.0
                mode = "parallel_wait"
                print("평행 주차위치 파악중... 2초)")
            else:
                try:
                    print("카메라 노드에 평행 주차 경로 서비스를 요청합니다...")
                    resp = get_parallel_parking_path_service()
                    if not resp.success:
                        print(f"서비스 실패: {resp.message}")
                    else:
                        print(f"서비스 성공: {resp.message}")
                        parking_csv_path = '/home/icas/parallel_parking_typeA.csv'
                        new_parking_x, new_parking_y = load_csv_xy(parking_csv_path)
                        PARALLEL_PARKING_START_IDX = 601
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

        # ── 신호등(직진) 대기: 빨간불 유지 동안 정지, 빨간불 해제되면 주행 재개
        elif is_straight_waiting:
            if tl_state == "red":
                velocity.linear.x = 0.0
                steering_command = 0.0
                mode = "straight_tl_wait"
                print("직진 신호등: 빨간불 → 정지 유지")
            else:
                print("직진 신호등: 빨간불 해제 → 주행 재개")
                if triggered_wait_idx in straight_traffic_list:
                    straight_traffic_list.remove(triggered_wait_idx)
                is_straight_waiting = False
                triggered_wait_idx = None
                mode = "fwd"

        # ── 신호등(좌회전) 대기: 'rl'(좌회전 화살표) 때만 출발, 나머지는 정지
        elif is_left_waiting:
            if tl_state != "rl":
                velocity.linear.x = 0.0
                steering_command = 0.0
                mode = "left_tl_wait"
                print("좌회전 신호등: 'rl' 아님 → 정지 유지")
            else:
                print("좌회전 신호등: 'rl' → 주행 재개")
                if triggered_wait_idx in left_traffic_list:
                    left_traffic_list.remove(triggered_wait_idx)
                is_left_waiting = False
                triggered_wait_idx = None
                mode = "fwd"

        # 후진 구간
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

        # 오르막 주행 구간
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

        # 슬로우 구간
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

        # 일반 주행
        else:
            mode = "fwd"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(current_yaw)
            )
            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            velocity.linear.x = 27
            velocity.angular.z = steering_command

        # 로그/퍼블리시
        print(f"현재 추종 웨이포인트: {traj.last_idx}, Mode: {mode}, Look-ahead(L): {current_L}")
        print(f"현재속도: {velocity.linear.x:.2f}, 현재 조향각: {velocity.angular.z:.2f}")
        print(f"목표점까지의 거리: {getDistance([control_x, control_y], target_point):.2f} | TL={tl_state}\n")

        log_rows.append({
            "time": rospy.get_time(),
            "gps_x": current_x, "gps_y": current_y,
            "control_x": control_x, "control_y": control_y,
            "yaw": current_yaw, "wp_idx": traj.last_idx, "mode": mode,
            "v_cmd": float(velocity.linear.x), "steer_cmd": float(velocity.angular.z),
            "tl_state": tl_state
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
