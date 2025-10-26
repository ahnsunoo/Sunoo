#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LidarObstacleDetector → 직사각형(축 정렬) 감지 영역 지원 + /emergency_stop 퍼블리시
- 장애물 감지 시 std_msgs/Bool(True)를 '/emergency_stop' 토픽으로 퍼블리시
- 미감지 시 False 퍼블리시 (디바운싱/홀드 적용)
- '직사각형 모드'를 켜면 부채꼴(FOV) 대신 차량 전방의 축정렬 직사각형(AABB) 영역으로 판정
  · 직사각형은 차량(센서) 원점을 (0,0), +x 전방, +y 좌측 기준
  · 파라미터로 직사각형의 시작거리(x 오프셋), 길이(x), 폭(y), (선택) y 중앙 오프셋을 설정
  · 각 빔에 대해 레이-직사각형 교차 구간 [t_entry, t_exit]을 계산하고, 측정 거리 d가 그 안이면 히트
  · 각도 마스크는 그대로 적용(필요 없으면 비우기)
"""

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool
import math

# =========================================================
#                 🔧 TUNABLE PARAMETERS 🔧
# ---------------------------------------------------------
# ※ 아래 기본값들은 code-level 기본값입니다.
#    동일한 키로 ROS Parameter가 주입되면 그 값으로 덮어씁니다.
#    (예: rosparam, yaml, launch 파일의 <param> 등)
# ---------------------------------------------------------

# [운영 로직] 장애물 검사를 수행할 웨이포인트 인덱스 목록
## DEFAULT_CHECK_WAYPOINTS = [range(0,200)*]
DEFAULT_CHECK_WAYPOINTS = [*range(765,840)]

# [거리 창] 유효 거리 범위(m)
DEFAULT_NEAR_LIMIT_M        = 0.01
DEFAULT_FAR_LIMIT_M         = 10.0
DEFAULT_DETECTION_DISTANCE  = 5.0 

# [시야(FOV)] 차량 정면 기준 시야각(deg)과 센서 장착 오프셋(deg)
#  (rect 모드가 꺼져 있을 때만 사용)
DEFAULT_FORWARD_ANGLE_RANGE_DEG = 30.0
DEFAULT_FRONT_YAW_OFFSET_DEG    = 0.0

# [섀도/가림] 차량 기준 마스크 섹터 목록(deg, 튜플 리스트)
DEFAULT_ANGLE_MASKS_DEG = [(90.0, 95.0), (120.0, 180.0)]

# [안정화] 단일빔 대신 '연속빔(clustering)'으로 트리거
DEFAULT_CLUSTER_MIN_BEAMS = 3

# [디바운싱(홀드)] 출력 플리커링 방지(초)
DEFAULT_HOLD_ON_SEC  = 0.3
DEFAULT_HOLD_OFF_SEC = 0.3

# [퍼블리시] 비상정지 신호 퍼블리시 대상 토픽명 (다른 노드가 구독)
DEFAULT_EMERGENCY_TOPIC = "/emergency_stop"

# === 직사각형 모드 파라미터 ===
#   - RECT_MODE_ENABLE: True면 직사각형 모드 사용(부채꼴 대신)
#   - RECT_X_OFFSET   : 원점(센서)에서 전방으로 떨어진 시작 x 위치 [m]
#   - RECT_LENGTH     : x 방향 길이 [m]  (직사각형은 [x0, x0+length])
#   - RECT_WIDTH      : y 방향 폭 [m]    (직사각형은 [yc - W/2, yc + W/2])
#   - RECT_Y_CENTER   : y 중앙 오프셋 [m] (기본 0 → 차선 중앙 정렬)
DEFAULT_RECT_MODE_ENABLE = True
DEFAULT_RECT_X_OFFSET    = 0.5
DEFAULT_RECT_LENGTH      = 2.0
DEFAULT_RECT_WIDTH       = 1.0
DEFAULT_RECT_Y_CENTER    = 0.0
# =========================================================


def wrap_deg(x):
    """[-180, 180)로 정규화"""
    return (x + 180.0) % 360.0 - 180.0


def in_sector(angle_deg, start_deg, end_deg):
    """경계 교차(예: 170 ~ -170)도 지원하는 각도 포함 검사 (모두 차량 기준)"""
    a = wrap_deg(angle_deg)
    s = wrap_deg(start_deg)
    e = wrap_deg(end_deg)
    if s <= e:
        return s <= a <= e
    else:
        # s..180 또는 -180..e
        return (a >= s) or (a <= e)


def _parse_angle_masks(raw):
    """
    마스크 파라미터 안전 파싱:
      - 리스트/튜플: [[-30,30],[150,210]] 또는 [(-30,30),(150,210)]
      - 문자열 CSV : "-30:30,150:210"
      - 문자열 리스트: ["-30~30","150~210"]
    반환: [(a0, a1), ...] (float 튜플)
    """
    out = []

    def push(a, b):
        try:
            out.append((float(a), float(b)))
        except Exception:
            pass

    if raw is None:
        return out

    if isinstance(raw, (list, tuple)):
        for it in raw:
            if isinstance(it, (list, tuple)) and len(it) == 2:
                push(it[0], it[1])
            elif isinstance(it, str):
                sep = "~" if "~" in it else (":" if ":" in it else None)
                if sep:
                    parts = it.split(sep)
                    if len(parts) == 2:
                        push(parts[0].strip(), parts[1].strip())
    elif isinstance(raw, str):
        for token in raw.split(","):
            token = token.strip()
            if not token:
                continue
            sep = "~" if "~" in token else (":" if ":" in token else None)
            if sep:
                parts = token.split(sep)
                if len(parts) == 2:
                    push(parts[0].strip(), parts[1].strip())
    return out


class LidarObstacleDetector:
    def __init__(self):
        rospy.init_node('lidar_obstacle_detector', anonymous=True)

        # ---------- ROS Param 로드(없으면 상단 기본값 사용) ----------
        self.obstacle_check_waypoints = rospy.get_param("~check_waypoints", DEFAULT_CHECK_WAYPOINTS)
        self.detection_distance       = float(rospy.get_param("~detection_distance", DEFAULT_DETECTION_DISTANCE))
        self.forward_angle_range_deg  = float(rospy.get_param("~forward_angle_range_deg", DEFAULT_FORWARD_ANGLE_RANGE_DEG))
        self.front_yaw_offset_deg     = float(rospy.get_param("~front_yaw_offset_deg", DEFAULT_FRONT_YAW_OFFSET_DEG))
        self.near_limit_m             = float(rospy.get_param("~near_limit_m", DEFAULT_NEAR_LIMIT_M))
        self.far_limit_m              = float(rospy.get_param("~far_limit_m", DEFAULT_FAR_LIMIT_M))
        self.cluster_min_beams        = int(rospy.get_param("~cluster_min_beams", DEFAULT_CLUSTER_MIN_BEAMS))
        self.hold_on_sec              = float(rospy.get_param("~hold_on_sec", DEFAULT_HOLD_ON_SEC))
        self.hold_off_sec             = float(rospy.get_param("~hold_off_sec", DEFAULT_HOLD_OFF_SEC))
        self.angle_masks_deg          = _parse_angle_masks(
            rospy.get_param("~angle_masks_deg", DEFAULT_ANGLE_MASKS_DEG)
        )
        self.emergency_topic          = rospy.get_param("~emergency_topic", DEFAULT_EMERGENCY_TOPIC)

        # 직사각형 모드 파라미터
        self.rect_mode_enable = bool(rospy.get_param("~rect_mode_enable", DEFAULT_RECT_MODE_ENABLE))
        self.rect_x_offset    = float(rospy.get_param("~rect_x_offset", DEFAULT_RECT_X_OFFSET))
        self.rect_length      = float(rospy.get_param("~rect_length", DEFAULT_RECT_LENGTH))
        self.rect_width       = float(rospy.get_param("~rect_width", DEFAULT_RECT_WIDTH))
        self.rect_y_center    = float(rospy.get_param("~rect_y_center", DEFAULT_RECT_Y_CENTER))

        # ---------- 내부 상태/IO ----------
        self.latest_scan = None
        self.active_zone = False  # WP 콜백으로 on/off
        self.latched_state = False
        self.last_state_change = rospy.Time.now()

        # emergency_stop 퍼블리셔 (다른 노드가 구독)
        self.emergency_pub = rospy.Publisher(self.emergency_topic, Bool, queue_size=1)

        rospy.Subscriber('/current_waypoint', Int32, self.waypoint_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        mode_str = "RECT" if self.rect_mode_enable else "FOV"
        rospy.loginfo(
            "LidarObstacleDetector ready: "
            f"mode={mode_str}, "
            f"WPs={self.obstacle_check_waypoints}, "
            f"FOV={self.forward_angle_range_deg:.1f} deg, "
            f"range_win=[{self.near_limit_m:.2f}, {self.far_limit_m:.2f}] m, "
            f"det_max={self.detection_distance:.2f} m, "
            f"cluster_min={self.cluster_min_beams}, "
            f"hold=(+{self.hold_on_sec:.2f}/-{self.hold_off_sec:.2f}) s, "
            f"masks={self.angle_masks_deg}, "
            f"pub->{self.emergency_topic}, "
            f"rect(x0={self.rect_x_offset:.2f}, L={self.rect_length:.2f}, W={self.rect_width:.2f}, yC={self.rect_y_center:.2f})"
        )

    # 웨이포인트 변화 시 검사 on/off 스위치
    def waypoint_callback(self, msg):
        idx = msg.data
        self.active_zone = (idx in self.obstacle_check_waypoints)
        if not self.active_zone:
            # 검사 구간이 아니면 즉시 False 발행(정지 해제)
            self._publish_stop(False, force=True)
            rospy.loginfo(f"WP#{idx}: not in check zone → {self.emergency_topic}=False.")
        else:
            rospy.loginfo(f"WP#{idx}: ACTIVE zone. Scanning each /scan.")

    # 검사 구간이면 매 스캔 프레임마다 검사
    def scan_callback(self, msg):
        self.latest_scan = msg
        if self.active_zone:
            self.check_for_obstacles()

    def _publish_stop(self, stop, force=False):
        """디바운스/홀드 적용 후 emergency_stop 퍼블리시"""
        now = rospy.Time.now()

        if force:
            self.latched_state = stop
            self.last_state_change = now
            self.emergency_pub.publish(Bool(data=stop))
            return

        if stop != self.latched_state:
            elapsed = (now - self.last_state_change).to_sec()
            if stop is True:
                # False -> True: 최근 미감지 상태를 최소 hold_off_sec 유지했는가?
                if elapsed < self.hold_off_sec:
                    return
            else:
                # True -> False: 최근 감지 상태를 최소 hold_on_sec 유지했는가?
                if elapsed < self.hold_on_sec:
                    return

            self.latched_state = stop
            self.last_state_change = now
            self.emergency_pub.publish(Bool(data=stop))

    # ----------------- 핵심: 레이-직사각형 교차(슬랩 방식) -----------------
    @staticmethod
    def _ray_rect_intersection_t(dir_x, dir_y, x0, x1, y0, y1, eps=1e-9):
        """
        원점(0,0)에서 방향 (dir_x, dir_y)로 나가는 반직선과
        축정렬 직사각형 [x0,x1]x[y0,y1]의 교차 구간 [t_entry, t_exit] (t>=0)을 반환.
        교차 없으면 None.
        """
        tmin = -float('inf')
        tmax =  float('inf')

        # X slabs
        if abs(dir_x) < eps:
            # 레이가 x에 평행 → 원점 x=0이 slab 안에 있어야 통과 가능
            if 0.0 < x0 or 0.0 > x1:
                return None
        else:
            tx1 = x0 / dir_x
            tx2 = x1 / dir_x
            tmin_x = min(tx1, tx2)
            tmax_x = max(tx1, tx2)
            tmin = max(tmin, tmin_x)
            tmax = min(tmax, tmax_x)

        # Y slabs
        if abs(dir_y) < eps:
            if 0.0 < y0 or 0.0 > y1:
                return None
        else:
            ty1 = y0 / dir_y
            ty2 = y1 / dir_y
            tmin_y = min(ty1, ty2)
            tmax_y = max(ty1, ty2)
            tmin = max(tmin, tmin_y)
            tmax = min(tmax, tmax_y)

        if tmax < max(tmin, 0.0):
            return None

        t_entry = max(tmin, 0.0)
        t_exit  = tmax
        return (t_entry, t_exit)

    def check_for_obstacles(self):
        scan = self.latest_scan
        if scan is None:
            rospy.logwarn_throttle(1.0, "LiDAR data not available yet.")
            return

        # 유효 거리 범위: 센서 스펙 ∩ 윈도우 ∩ detection_distance (상한)
        rmin = max(scan.range_min, self.near_limit_m)
        rmax = min(scan.range_max, self.far_limit_m, self.detection_distance)

        cluster = 0
        hit_info = None  # (d, i, body_deg)

        # FOV 경계(직사각형 모드=ON이어도 마스크 처리 위해 body 각 계산은 수행)
        half = 180.0 if self.forward_angle_range_deg >= 359.9 else self.forward_angle_range_deg / 2.0
        fov_start = -half
        fov_end   =  half

        # 직사각형 경계(차량/body 기준)
        if self.rect_mode_enable:
            x0 = self.rect_x_offset
            x1 = self.rect_x_offset + self.rect_length
            y0 = self.rect_y_center - self.rect_width * 0.5
            y1 = self.rect_y_center + self.rect_width * 0.5
            # 안전 차단: x0<x1 보장
            if x1 < x0:
                x0, x1 = x1, x0
            if y1 < y0:
                y0, y1 = y1, y0

        # 전체 빔 순회
        for i, d in enumerate(scan.ranges):
            # 일부 드라이버는 미측정에 0.0/inf/Nan을 줌 → 배제
            if not math.isfinite(d) or d <= 0.0 or d < rmin or d > rmax:
                cluster = 0
                continue

            angle_i    = scan.angle_min + i * scan.angle_increment  # [rad], 센서 기준
            sensor_deg = math.degrees(angle_i)
            body_deg   = wrap_deg(sensor_deg - self.front_yaw_offset_deg)

            # 각도 마스크(차량 기준). 직사각형 모드에서도 적용(원치 않으면 파라미터 비우기)
            if any(in_sector(body_deg, a0, a1) for (a0, a1) in self.angle_masks_deg):
                cluster = 0
                continue

            if self.rect_mode_enable:
                # 레이-직사각형 교차
                dir_x = math.cos(math.radians(body_deg))
                dir_y = math.sin(math.radians(body_deg))
                hit = self._ray_rect_intersection_t(dir_x, dir_y, x0, x1, y0, y1)
                if hit is None:
                    cluster = 0
                    continue
                t_entry, t_exit = hit

                # 실제 측정 d가 직사각형 내 교차구간에 들어오면 히트
                # 또한 rmin/rmax 창과도 교집합
                lo = max(rmin, t_entry)
                hi = min(rmax, t_exit)
                if not (lo <= d <= hi):
                    cluster = 0
                    continue
            else:
                # === 기존 부채꼴(FOV) 방식 ===
                if not in_sector(body_deg, fov_start, fov_end):
                    cluster = 0
                    continue

            # 유효 히트
            cluster += 1
            if cluster >= self.cluster_min_beams:
                hit_info = (d, i, body_deg)
                break

        if hit_info is not None:
            d, i, body_deg = hit_info
            # 장애물 감지 → 비상정지 True
            self._publish_stop(True)
            shape = "RECT" if self.rect_mode_enable else "FOV"
            rospy.logwarn_throttle(
                0.5, f"[{shape}] STOP=True: obstacle ~{d:.2f} m (beam {i}, body {body_deg:.1f}°)."
            )
        else:
            # 미감지 → 비상정지 False
            self._publish_stop(False)
            shape = "RECT" if self.rect_mode_enable else "FOV"
            if self.rect_mode_enable:
                rospy.loginfo_throttle(
                    1.0,
                    f"[{shape}] STOP=False: clear in rect x=[{self.rect_x_offset:.2f},{(self.rect_x_offset+self.rect_length):.2f}] m, "
                    f"y=[{(self.rect_y_center - self.rect_width*0.5):.2f},{(self.rect_y_center + self.rect_width*0.5):.2f}] m."
                )
            else:
                rospy.loginfo_throttle(
                    1.0,
                    f"[{shape}] STOP=False: clear in [{rmin:.2f}, {rmax:.2f}] m, FOV({self.forward_angle_range_deg:.1f}°)."
                )


if __name__ == '__main__':
    try:
        node = LidarObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

