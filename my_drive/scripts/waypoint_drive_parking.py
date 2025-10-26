#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_waypoint_controller (주행 노드; 전진/후진 로직은 사용자가 준 코드와 동일화)

개요
- 기본(좌측) 경로 CSV(WAYPOINTS_FILE)를 따라 주행(Pure-Pursuit)합니다.
- 판단 웨이포인트(DECIDE_WAIT_IDX)에 도달하면 /parking/decide(Trigger) 서비스를 호출하여
  빈칸 방향(LEFT/RIGHT/UNKNOWN)을 받아, RIGHT인 경우 주차 구간 [PARK_START_IDX..PARK_END_IDX]의
  웨이포인트(x, y)를 우측 전용 CSV(RIGHT_PATCH_FILE)로 부분 교체합니다.
- 주행/후진 제어는 "사용자가 제공한 코드"의 수식/분기와 동일:
  * Lookahead L 기본값 3.0, litte_L_point 구간에서만 L=1.0
  * slow_point 구간 속도 20, 일반 전진 속도 27
  * back_point 구간에서 후진: yaw+180° 래핑 + 조향부호(-1) 처리 + 속도 -20
- 한글 상태 출력(현재 타깃 WP, 최근접 WP, 모드, L, 속도/조향, 타깃거리),
  "주차구역 판단위치 도달"·"판단 결과" 메시지를 콘솔에 표시.

변경 사항(요청 반영)
- right.csv에 idx가 없어도 동작: load_right_patch()가 idx 유무를 자동으로 처리
- (보너스) CSV 컬럼명 대소문자 무시: X,Y,Idx 등도 허용
"""

import math
import os
from datetime import datetime
import pandas as pd

import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Trigger


# ==============================
# 경로/판단 파라미터
# ==============================
WAYPOINTS_FILE   = '/home/icas/left.csv'     # 기본 전체 코스 CSV (x,y 헤더면 idx 없어도 OK)
RIGHT_PATCH_FILE = '/home/icas/right.csv'    # 우측 주차용 패치 CSV (idx 있으면 범위 필터, 없으면 x,y 마지막 need개 사용)

DECIDE_WAIT_IDX  = 12    # 판단/정지 웨이포인트(0-based: 13번째에서 판단)
PARK_START_IDX   = 13    # 주차 구간 시작 인덱스 (부분 교체 시작)
PARK_END_IDX     = 17    # 주차 구간 끝 인덱스   (부분 교체 끝)

# ==============================
# 주행/제어 파라미터 (사용자 코드와 동일화)
# ==============================
wheelbase   = 0.73      # 축간 거리(전륜-후륜)
L_default   = 3.0       # 전방 주시거리 기본값(L)
kp_angular  = 1.0       # 조향 명령 스케일(도 단위)

REAR_AXLE_OFFSET = 0.73 # GPS 안테나 ↔ 뒷바퀴 축 거리(후진 시 제어 기준 보정)

# 구간별 인덱스 세트(사용자가 준 코드와 동일)
litte_L_point  = [516, 517, 518, 519, 520, 521, 522]  # L=1.0으로 줄여 접근할 WP
slow_point     = [516, 517, 518, 519, 520, 521]       # 속도를 20으로 제한할 WP
back_point     = [518, 519]                           # 후진으로 이동할 WP
wait_point     = [519]                                # 평지 정지 웨이포인트(옵션; 여기선 사용 안 함)
uphill_wait_point  = []                                # 오르막 정지(옵션)
uphill_drive_point = []                                # 오르막 출력 상승(옵션)

# ==============================
# 상태 변수
# ==============================
current_x = 0.0       # 현재 위치 x
current_y = 0.0       # 현재 위치 y
current_yaw = 0.0     # 현재 요(heading), 라디안 유지

velocity = Twist()    # /gps_cmd_vel 퍼블리시할 속도/조향 명령
parking_decided = False  # 판단 1회만 수행


# ==============================
# 콜백/유틸
# ==============================
def gps_callback(msg: Pose2D):
    """ /gps/utm_pos1 콜백: x,y 갱신 """
    global current_x, current_y
    current_x = msg.x
    current_y = msg.y

def imu_callback(msg: Float32):
    """ /imu/yaw 콜백: deg → rad 변환하여 yaw 갱신 """
    global current_yaw
    current_yaw = math.radians(msg.data)

def getDistance(p1, p2):
    """ 두 점 p1=[x1,y1], p2=[x2,y2] 사이 유클리드 거리 """
    dx, dy = p1[0] - p2[0], p1[1] - p2[1]
    return math.hypot(dx, dy)

def nearest_waypoint_index(traj: 'Trajectory', x: float, y: float, hint_idx: int = None, win: int = 50):
    """
    현재 위치에서 가장 가까운 웨이포인트 인덱스를 찾는다.
    - 속도 확보를 위해 hint_idx±win 범위로 제한 가능(기본 50)
    """
    n = len(traj.traj_x)
    if n == 0:
        return 0, float('inf')

    if hint_idx is None:
        idx_range = range(n)
    else:
        s = max(0, hint_idx - win)
        e = min(n, hint_idx + win + 1)
        idx_range = range(s, e)

    best_idx, best_dist = 0, float('inf')
    for i in idx_range:
        dx = traj.traj_x[i] - x
        dy = traj.traj_y[i] - y
        d = math.hypot(dx, dy)
        if d < best_dist:
            best_dist, best_idx = d, i
    return best_idx, best_dist

def _find_col(df: pd.DataFrame, *candidates):
    """
    CSV 컬럼명 찾기(대소문자 무시). 예: _find_col(df,'x') → 'x' 또는 'X' 등 실제 컬럼명 반환
    """
    lower_map = {c.lower(): c for c in df.columns}
    for name in candidates:
        if name.lower() in lower_map:
            return lower_map[name.lower()]
    return None


# ==============================
# Trajectory: 목표점 탐색 + 구간 패치
# ==============================
class Trajectory:
    def __init__(self, traj_x, traj_y):
        """
        traj_x, traj_y : 전체 웨이포인트 x/y 리스트 (동일 길이)
        last_idx       : Pure-Pursuit이 마지막으로 선택한 타깃 인덱스
        """
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0

    def getPoint(self, idx):
        """ 인덱스에 해당하는 [x, y] 포인트 반환 """
        return [self.traj_x[idx], self.traj_y[idx]]

    def getTargetPoint(self, pos, look_ahead_dist):
        """
        last_idx에서 시작하여 pos와 look_ahead_dist 이상 떨어진 지점까지 인덱스를 전진.
        """
        target_idx   = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist    = getDistance(pos, target_point)

        while curr_dist < look_ahead_dist and target_idx < len(self.traj_x) - 1:
            target_idx   += 1
            target_point  = self.getPoint(target_idx)
            curr_dist     = getDistance(pos, target_point)

        self.last_idx = target_idx
        return self.getPoint(target_idx)

    def patch_segment(self, start_idx, end_idx, new_x, new_y):
        """
        [start_idx..end_idx] 구간 좌표를 새로운 좌표(new_x/new_y)로 교체.
        교체 구간 안에서 이미 주행 중(last_idx 범위 내)이면 last_idx를 start_idx-1로 되감아
        재탐색 여유를 확보.
        """
        n = end_idx - start_idx + 1
        self.traj_x[start_idx:end_idx+1] = new_x[:n]
        self.traj_y[start_idx:end_idx+1] = new_y[:n]

        if start_idx <= self.last_idx <= end_idx:
            self.last_idx = max(start_idx - 1, 0)


# ==============================
# 패치 CSV 로더 / 판단 서비스 호출
# ==============================
def load_right_patch(csv_path, start_idx, end_idx):
    """
    우측 주차용 패치 CSV 로드 (idx 유무 자동 대응)
    - (권장) idx,x,y 3컬럼이 있으면: idx ∈ [start_idx..end_idx]만 필터링하여 사용
    - (대안) idx가 없고 x,y만 있으면:
        * x,y 컬럼에서 NaN/빈칸 제거 후, 유효한 행들 중 '마지막 need개' 사용
        * need = end_idx - start_idx + 1  (예: 13~17 → 5개)
    - 컬럼명 대소문자 무시: x/X, y/Y, idx/IDX 모두 허용
    """
    df = pd.read_csv(csv_path)

    col_idx = _find_col(df, 'idx', 'index')
    col_x   = _find_col(df, 'x')
    col_y   = _find_col(df, 'y')

    if col_x is None or col_y is None:
        raise ValueError(f"[patch] '{csv_path}'에서 x,y 컬럼을 찾지 못했습니다. 헤더를 'x,y'로 맞춰주세요.")

    need = end_idx - start_idx + 1

    if col_idx is not None:
        # ── 케이스 A: idx가 있을 때 (기존 동작) ──
        seg = df[(df[col_idx] >= start_idx) & (df[col_idx] <= end_idx)][[col_x, col_y]].dropna().copy()
        if len(seg) != need:
            rospy.logwarn(f"[patch] idx기반 추출 개수={len(seg)} (필요 {need}). 가능한 만큼 사용합니다.")
            # 혹시 섞여 있으면 정렬
            seg = df[(df[col_idx] >= start_idx) & (df[col_idx] <= end_idx)].sort_values(col_idx)[[col_x, col_y]].dropna().copy()
            # 개수가 많으면 뒤에서 need개만
            if len(seg) > need:
                seg = seg.tail(need)
    else:
        # ── 케이스 B: idx가 없을 때 (x,y만 존재) ──
        seg = df[[col_x, col_y]].dropna().copy()
        if len(seg) < need:
            rospy.logwarn(f"[patch] right.csv 유효 행이 {len(seg)}개입니다. {need}개 필요합니다. 가능한 만큼만 사용합니다.")
            # 부족하면 있는 만큼만 사용(패치 범위의 앞부분만 채워짐)
        elif len(seg) > need:
            # 파일 끝쪽 좌표가 최신/의도 구간이라고 가정하고 '마지막 need개' 사용
            seg = seg.tail(need)

    new_x = seg[col_x].astype(float).tolist()
    new_y = seg[col_y].astype(float).tolist()
    return new_x, new_y


def call_parking_decision(timeout_s=6.0):
    """
    /parking/decide (Trigger) 호출
    - 응답 message의 첫 단어: 'LEFT' | 'RIGHT' | 'UNKNOWN'
    """
    srv_name = '/parking/decide'
    try:
        rospy.wait_for_service(srv_name, timeout=timeout_s)
        proxy = rospy.ServiceProxy(srv_name, Trigger)
        resp  = proxy()
        side  = (resp.message.split()[0].strip() if resp.message else 'UNKNOWN')
        rospy.loginfo(f"[gps] /parking/decide 응답: {side} (success={resp.success})")
        return side if resp.success else 'UNKNOWN'
    except Exception as e:
        rospy.logwarn(f"[gps] 서비스 호출 실패: {e}")
        return 'UNKNOWN'


# ==============================
# 메인
# ==============================
def main():
    global parking_decided

    rospy.init_node('gps_waypoint_controller')
    rate = rospy.Rate(10)  # 10Hz

    # 로그 저장 디렉토리 준비
    stamp  = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    logdir = os.path.expanduser(f"/home/icas/{stamp}")
    os.makedirs(logdir, exist_ok=True)
    log_rows = []

    # 웨이포인트 로드 (x,y 컬럼 필요; idx 불필요)
    try:
        df_wp = pd.read_csv(WAYPOINTS_FILE)
        col_x = _find_col(df_wp, 'x'); col_y = _find_col(df_wp, 'y')
        if col_x is None or col_y is None:
            raise ValueError("WAYPOINTS_FILE에 x,y 컬럼이 없습니다. 헤더를 'x,y'로 맞춰주세요.")
        traj_x = df_wp[col_x].astype(float).tolist()
        traj_y = df_wp[col_y].astype(float).tolist()
    except Exception as e:
        rospy.logerr(f"Waypoint CSV 읽기 실패: {e}")
        return
    if not traj_x:
        rospy.logerr("Waypoint list is empty.")
        return
    traj = Trajectory(traj_x, traj_y)

    # 퍼블리셔/서브스크라이버
    vel_pub  = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)
    wp_pub   = rospy.Publisher('/current_waypoint', Int32, queue_size=1, latch=True)
    rospy.Subscriber('/gps/utm_pos1', Pose2D, gps_callback)
    rospy.Subscriber('/imu/yaw',      Float32, imu_callback)
    rospy.sleep(0.2)

    while not rospy.is_shutdown():
        # -----------------------------
        # 제어 기준점: 후진 구간이면 뒷바퀴축 기준으로 보정(사용자 코드와 동일)
        # -----------------------------
        if traj.last_idx in back_point:
            control_x = current_x - REAR_AXLE_OFFSET * math.cos(current_yaw)
            control_y = current_y - REAR_AXLE_OFFSET * math.sin(current_yaw)
        else:
            control_x = current_x
            control_y = current_y

        # -----------------------------
        # Lookahead L 선택(사용자 코드와 동일)
        # -----------------------------
        if traj.last_idx in litte_L_point:
            current_L = 1.0
        else:
            current_L = L_default

        # -----------------------------
        # Pure-Pursuit 타깃 웨이포인트 계산
        # -----------------------------
        target_point = traj.getTargetPoint([control_x, control_y], current_L)
        wp_pub.publish(Int32(data=traj.last_idx))

        mode = "fwd"
        steering_command = 0.0

        # -----------------------------
        # 판단 웨이포인트 도달 시 1회만 서비스 호출
        # -----------------------------
        if (traj.last_idx == DECIDE_WAIT_IDX) and (not parking_decided):
            # 1. 즉시 정지 명령을 보내고 현재 시간을 기록
            velocity.linear.x  = 0.0
            velocity.angular.z = 0.0
            vel_pub.publish(velocity)

            print("🅿️  주차구역 판단위치 도달 — 5초간 정지하며 비전 판단을 시작합니다.")
            rospy.loginfo("[gps] DECIDE_WAIT_IDX 도달 → 5초 정지 및 비전 판단 시작")
            wait_start_time = rospy.get_time()

            # 2. 서비스 호출 (차량은 이 시간 동안 계속 정지 상태)
            #    타임아웃은 최대 정지 시간인 5초로 설정하는 것이 좋습니다.
            empty_side = call_parking_decision(timeout_s=5.0)
            print(f"🅿️  주차구역 판단 결과: {empty_side.lower()}")

            # 3. 서비스 호출이 끝난 후, 5초에서 소요된 시간을 뺀 나머지 시간만큼 추가 대기
            elapsed_time = rospy.get_time() - wait_start_time
            remaining_time = 5.0 - elapsed_time

            if remaining_time > 0:
                rospy.loginfo(f"[gps] 서비스 처리 완료. 남은 시간 {remaining_time:.2f}초 동안 추가 정지합니다.")
                rospy.sleep(remaining_time)

            # 4. 판단 결과에 따라 경로 교체 (기존과 동일)
            if empty_side == 'RIGHT':
                try:
                    new_x, new_y = load_right_patch(RIGHT_PATCH_FILE, PARK_START_IDX, PARK_END_IDX)
                    traj.patch_segment(PARK_START_IDX, PARK_END_IDX, new_x, new_y)
                    rospy.loginfo("[gps] 빈칸=RIGHT → 주차 구간을 RIGHT 경로로 교체 완료")
                except Exception as e:
                    rospy.logwarn(f"[gps] RIGHT 패치 로드 실패: {e} → 기본(LEFT) 유지")
            elif empty_side == 'LEFT':
                rospy.loginfo("[gps] 빈칸=LEFT → 기본(LEFT) 유지")
            else:
                rospy.logwarn("[gps] 판단 불확실/실패 → 기본(LEFT) 유지")

            parking_decided = True  # 판단은 한 번만 수행

        # -----------------------------
        # 주행/후진 로직 (사용자 코드와 동일한 수식/분기)
        # -----------------------------
        if traj.last_idx in back_point:
            # ===== 후진 모드 =====
            mode = "back"

            # 목표점의 절대방향(도)
            target_angle = math.degrees(math.atan2(
                target_point[1] - control_y,
                target_point[0] - control_x
            ))
            # 현재 헤딩(도) → 후진 시 효과 반영: +180 래핑
            current_angle_deg = math.degrees(current_yaw)
            current_angle_deg = (current_angle_deg + 180) % 360

            # 차각(diff) 계산 후 -180~+180도로 wrap
            diff_angle = target_angle - current_angle_deg
            steering_angle = ((diff_angle + 180) % 360) - 180

            # 사용자 코드와 동일: 조향 부호 반전
            steering_angle = (-1) * steering_angle
            steering_command = steering_angle * kp_angular

            # 후진 속도
            velocity.linear.x  = -20.0
            velocity.angular.z = steering_command

        elif traj.last_idx in uphill_drive_point:
            # ===== 오르막 가속 모드(옵션) =====
            mode = "uphill_drive"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(current_yaw)
            )
            steering_angle   = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            velocity.linear.x  = 50.0
            velocity.angular.z = steering_command

        elif traj.last_idx in slow_point:
            # ===== 저속 구간 =====
            mode = "slow"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(current_yaw)
            )
            steering_angle   = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            velocity.linear.x  = 20.0
            velocity.angular.z = steering_command

        else:
            # ===== 일반 전진 모드 =====
            mode = "fwd"
            alpha = math.atan2(
                (target_point[1] - control_y) * math.cos(current_yaw) - (target_point[0] - control_x) * math.sin(current_yaw),
                (target_point[0] - control_x) * math.cos(current_yaw) + (target_point[1] - control_y) * math.sin(current_yaw)
            )
            steering_angle   = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), current_L))
            steering_command = steering_angle * kp_angular
            velocity.linear.x  = 27.0
            velocity.angular.z = steering_command

        # -----------------------------
        # 상태/로그 출력 (한글)
        # -----------------------------
        dist_tp = getDistance([control_x, control_y], target_point)
        rospy.loginfo_throttle(
            0.5,
            f"[{mode}] wp:{traj.last_idx} L:{current_L:.1f} v:{velocity.linear.x:.1f} "
            f"steer:{velocity.angular.z:.1f} dist:{dist_tp:.2f}"
        )

        near_idx, near_dist = nearest_waypoint_index(traj, control_x, control_y, hint_idx=traj.last_idx, win=50)
        print(f"현재 추종 웨이포인트(타깃): {traj.last_idx} | 현재 위치 기준 가장 가까운 웨이포인트: {near_idx} (거리 {near_dist:.2f} m) | 모드: {mode} | Look-ahead L: {current_L:.1f}")
        print(f"현재 속도: {velocity.linear.x:.2f} | 현재 조향명령: {velocity.angular.z:.2f}")
        print(f"목표점까지의 거리: {dist_tp:.2f} m\n")

        # 퍼블리시 + 로그 적재
        vel_pub.publish(velocity)
        log_rows.append({
            "time": rospy.get_time(),
            "gps_x": current_x, "gps_y": current_y,
            "control_x": control_x, "control_y": control_y,
            "yaw": current_yaw, "wp_idx": traj.last_idx, "mode": mode,
            "v_cmd": float(velocity.linear.x), "steer_cmd": float(velocity.angular.z)
        })

        rate.sleep()

    # 안전 종료: 정지 명령 1회 송신
    velocity.linear.x  = 0.0
    velocity.angular.z = 0.0
    try:
        vel_pub.publish(velocity)
    except Exception:
        pass

    # 주행 로그 저장
    try:
        df_log = pd.DataFrame(log_rows)
        out_csv = os.path.join(logdir, "track.csv")
        df_log.to_csv(out_csv, index=False, encoding="utf-8")
        print(f"[LOG] 주행 궤적 저장 완료: {out_csv}")
    except Exception as e:
        rospy.logwarn(f"주행 로그 저장 실패: {e}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

