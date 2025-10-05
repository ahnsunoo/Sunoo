#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LidarObstacleDetector → 직사각형 감지 + 동일 높이(y-band) 강화 + /emergency_stop 퍼블리시 + RViz 시각화
- 장애물 감지 시 std_msgs/Bool(True)를 '/emergency_stop' 토픽으로 퍼블리시
- 미감지 시 False 퍼블리시 (디바운싱/홀드 적용)
- 직사각형 모드: 차량 전방 축정렬 직사각형(AABB)으로 판정 (ray-rect 교차)
- 동일 높이(y-band) 필터: 특정 y 중심선 주변(±tolerance)만 강화 감지
- 시간 누적(window) 투표: 최근 N프레임 중 K회 이상 히트 시 True
- RViz 시각화:
  · 직사각형 영역: CUBE Marker (~rect_marker)
  · 히트 포인트(주로 y-밴드 통과점): POINTS Marker (~hits_marker)
"""

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from collections import deque

# =========================================================
#                 🔧 TUNABLE PARAMETERS 🔧
# ---------------------------------------------------------
# ※ 아래 기본값들은 code-level 기본값입니다.
#    동일한 키로 ROS Parameter가 주입되면 그 값으로 덮어씁니다.
#    (예: rosparam, yaml, launch 파일의 <param> 등)
# ---------------------------------------------------------

# [운영 로직] 장애물 검사를 수행할 웨이포인트 인덱스 목록
DEFAULT_CHECK_WAYPOINTS = [3, 7, 12]

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
#   - RECT_X_OFFSET   : 원점(센서/차량)에서 전방으로 떨어진 시작 x 위치 [m]
#   - RECT_LENGTH     : x 방향 길이 [m]  (직사각형은 [x0, x0+length])
#   - RECT_WIDTH      : y 방향 폭 [m]    (직사각형은 [yc - W/2, yc + W/2])
#   - RECT_Y_CENTER   : y 중앙 오프셋 [m] (기본 0 → 차량 중심 정렬)
DEFAULT_RECT_MODE_ENABLE = True
DEFAULT_RECT_X_OFFSET    = 0.5
DEFAULT_RECT_LENGTH      = 2.0
DEFAULT_RECT_WIDTH       = 1.0
DEFAULT_RECT_Y_CENTER    = 0.0

# === 동일 높이(y-band) 전용 파라미터 ===
DEFAULT_SAME_Y_ENABLE         = True           # y-밴드 필터 사용
DEFAULT_SAME_Y_TARGETS        = [0.0]          # 감지하고 싶은 y 중심선 리스트(m)
DEFAULT_SAME_Y_TOLERANCE      = 0.12           # 각 y 중심선에 대한 ±허용폭(m)
DEFAULT_X_MARGIN              = 0.10           # x 범위 여유(m) - 레이/양자화 오차 보정

# === 시간 누적(윈도우) 파라미터 ===
DEFAULT_WINDOW_FRAMES         = 6              # 최근 N프레임 누적
DEFAULT_MIN_HITS_IN_WINDOW    = 3              # N프레임 중 히트가 K회 이상이면 True
DEFAULT_MIN_POINTS_PER_FRAME  = 2              # 프레임당 최소 히트 포인트 수

# === RViz 시각화 파라미터 ===
DEFAULT_VIZ_ENABLE       = True
DEFAULT_VIZ_FRAME_ID     = "base_link"  # body 기준 계산값을 표시할 프레임
DEFAULT_VIZ_Z            = 0.05         # 마커 Z 높이 (바닥 위)
DEFAULT_VIZ_RECT_Z       = 0.02         # 직사각형 CUBE 두께
DEFAULT_VIZ_POINT_SIZE   = 0.07         # 히트 포인트 점 크기 (x=y)
DEFAULT_VIZ_RECT_ALPHA   = 0.25         # 직사각형 투명도 (0~1)
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

        # --- 동일 높이(y-밴드) 파라미터 ---
        self.same_y_enable    = bool(rospy.get_param("~same_y_enable", DEFAULT_SAME_Y_ENABLE))
        self.same_y_targets   = list(rospy.get_param("~same_y_targets", DEFAULT_SAME_Y_TARGETS))
        self.same_y_tol       = float(rospy.get_param("~same_y_tolerance", DEFAULT_SAME_Y_TOLERANCE))
        self.x_margin         = float(rospy.get_param("~x_margin", DEFAULT_X_MARGIN))

        # --- 시간 누적(윈도우) 파라미터 ---
        self.window_frames      = int(rospy.get_param("~window_frames", DEFAULT_WINDOW_FRAMES))
        self.min_hits_in_window = int(rospy.get_param("~min_hits_in_window", DEFAULT_MIN_HITS_IN_WINDOW))
        self.min_pts_per_frame  = int(rospy.get_param("~min_points_per_frame", DEFAULT_MIN_POINTS_PER_FRAME))
        self.hit_window = deque(maxlen=self.window_frames)  # 최근 프레임 히트 여부(0/1)

        # 시각화 파라미터
        self.viz_enable     = bool(rospy.get_param("~viz_enable", DEFAULT_VIZ_ENABLE))
        self.viz_frame_id   = rospy.get_param("~viz_frame_id", DEFAULT_VIZ_FRAME_ID)
        self.viz_z          = float(rospy.get_param("~viz_z", DEFAULT_VIZ_Z))
        self.viz_rect_z     = float(rospy.get_param("~viz_rect_z", DEFAULT_VIZ_RECT_Z))
        self.viz_point_size = float(rospy.get_param("~viz_point_size", DEFAULT_VIZ_POINT_SIZE))
        self.viz_rect_alpha = float(rospy.get_param("~viz_rect_alpha", DEFAULT_VIZ_RECT_ALPHA))

        # ---------- 내부 상태/IO ----------
        self.latest_scan = None
        self.active_zone = False  # WP 콜백으로 on/off
        self.latched_state = False
        self.last_state_change = rospy.Time.now()

        # 퍼블리셔
        self.emergency_pub = rospy.Publisher(self.emergency_topic, Bool, queue_size=1)

        # RViz markers
        self.rect_marker_pub = rospy.Publisher("~rect_marker", Marker, queue_size=1, latch=True)
        self.hits_marker_pub = rospy.Publisher("~hits_marker", Marker, queue_size=1)

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
            f"rect(x0={self.rect_x_offset:.2f}, L={self.rect_length:.2f}, W={self.rect_width:.2f}, yC={self.rect_y_center:.2f}), "
            f"same_y={self.same_y_enable}(targets={self.same_y_targets}, tol=±{self.same_y_tol:.2f}), "
            f"window(N={self.window_frames}, K={self.min_hits_in_window}, per_frame_pts>={self.min_pts_per_frame}), "
            f"viz={self.viz_enable}({self.viz_frame_id})"
        )

        # 최초 직사각형 마커 1회 송출(RECT 모드 & 시각화 on)
        if self.viz_enable and self.rect_mode_enable:
            self._publish_rect_marker()

    # 웨이포인트 변화 시 검사 on/off 스위치
    def waypoint_callback(self, msg):
        idx = msg.data
        self.active_zone = (idx in self.obstacle_check_waypoints)
        if not self.active_zone:
            # 검사 구간이 아니면 즉시 False 발행(정지 해제) 및 히트 점 삭제
            self._publish_stop(False, force=True)
            if self.viz_enable:
                self._clear_hits_marker()
            rospy.loginfo(f"WP#{idx}: not in check zone → {self.emergency_topic}=False.")
        else:
            rospy.loginfo(f"WP#{idx}: ACTIVE zone. Scanning each /scan.")

    # 검사 구간이면 매 스캔 프레임마다 검사
    def scan_callback(self, msg):
        self.latest_scan = msg
        if self.active_zone:
            self.check_for_obstacles()
        else:
            # 활성구간이 아니면 히트 점은 숨김
            if self.viz_enable:
                self._clear_hits_marker()

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

    # ----------------- 레이-직사각형 교차(슬랩 방식) -----------------
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

    # ----------------- RViz 마커 퍼블리시 -----------------
    def _publish_rect_marker(self):
        if not (self.viz_enable and self.rect_mode_enable):
            return

        x0 = self.rect_x_offset
        x1 = self.rect_x_offset + self.rect_length
        y0 = self.rect_y_center - self.rect_width * 0.5
        y1 = self.rect_y_center + self.rect_width * 0.5
        if x1 < x0: x0, x1 = x1, x0
        if y1 < y0: y0, y1 = y1, y0

        m = Marker()
        m.header.frame_id = self.viz_frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "rect"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        # 직사각형 중심/스케일
        m.pose.position.x = (x0 + x1) * 0.5
        m.pose.position.y = (y0 + y1) * 0.5
        m.pose.position.z = self.viz_z
        m.pose.orientation.w = 1.0
        m.scale.x = abs(x1 - x0)
        m.scale.y = abs(y1 - y0)
        m.scale.z = self.viz_rect_z
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = self.viz_rect_alpha
        self.rect_marker_pub.publish(m)

    def _publish_hits_marker(self, points_xy):
        if not self.viz_enable:
            return
        m = Marker()
        m.header.frame_id = self.viz_frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "hits"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.viz_point_size
        m.scale.y = self.viz_point_size
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0   # 노란색 점
        m.color.a = 1.0
        m.points = []
        for (x, y) in points_xy:
            p = Point()
            p.x = x
            p.y = y
            p.z = self.viz_z
            m.points.append(p)
        self.hits_marker_pub.publish(m)

    def _clear_hits_marker(self):
        if not self.viz_enable:
            return
        m = Marker()
        m.header.frame_id = self.viz_frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "hits"
        m.id = 0
        m.action = Marker.DELETE
        self.hits_marker_pub.publish(m)

    # ----------------- 메인 감지 루프 -----------------
    def check_for_obstacles(self):
        scan = self.latest_scan
        if scan is None:
            rospy.logwarn_throttle(1.0, "LiDAR data not available yet.")
            return

        # 유효 거리 범위: 센서 스펙 ∩ 윈도우 ∩ detection_distance (상한)
        rmin = max(scan.range_min, self.near_limit_m)
        rmax = min(scan.range_max, self.far_limit_m, self.detection_distance)

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
            if x1 < x0: x0, x1 = x1, x0
            if y1 < y0: y0, y1 = y1, y0
            # x 여유(양자화/반사오차 보정)
            x0 -= self.x_margin
            x1 += self.x_margin

        # --------- 프레임 누적을 위한 카운터들 ---------
        cluster = 0
        cluster_hit_found = False
        frame_band_points = []  # y-밴드로 통과한 포인트(시각화)
        points_in_rect = []     # 직사각형 교차로 통과한 포인트(시각화)

        # 전체 빔 순회
        for i, d in enumerate(scan.ranges):
            # 일부 드라이버는 미측정에 0.0/inf/Nan을 줌 → 배제
            if not math.isfinite(d) or d <= 0.0 or d < rmin or d > rmax:
                cluster = 0
                continue

            angle_i    = scan.angle_min + i * scan.angle_increment  # [rad], 센서 기준
            sensor_deg = math.degrees(angle_i)
            body_deg   = wrap_deg(sensor_deg - self.front_yaw_offset_deg)

            # 각도 마스크(차량 기준)
            if any(in_sector(body_deg, a0, a1) for (a0, a1) in self.angle_masks_deg):
                cluster = 0
                continue

            # 레이 방향 벡터
            dir_x = math.cos(math.radians(body_deg))
            dir_y = math.sin(math.radians(body_deg))

            # 좌표로 투영(차량/body 좌표계)
            px = d * dir_x
            py = d * dir_y

            # 직사각형 모드라면 AABB 범위 체크
            in_rect = True
            if self.rect_mode_enable:
                if not (x0 <= px <= x1):
                    in_rect = False
                if not (y0 <= py <= y1):
                    in_rect = False

            # --- 직사각형 교차(엄격 체크) ---
            rect_ray_ok = False
            if in_rect and self.rect_mode_enable:
                hit = self._ray_rect_intersection_t(dir_x, dir_y, x0, x1, y0, y1)
                if hit is not None:
                    t_entry, t_exit = hit
                    lo = max(rmin, t_entry)
                    hi = min(rmax, t_exit)
                    rect_ray_ok = (lo <= d <= hi)
                    if rect_ray_ok:
                        points_in_rect.append((px, py))

            # --- FOV 모드 클러스터(직사각형 OFF일 때만 유효) ---
            # 직사각형 모드에서도 보조로 쓰고 싶다면 조건을 조정하세요.
            fov_ok = False
            if not self.rect_mode_enable:
                if in_sector(body_deg, fov_start, fov_end):
                    fov_ok = True

            # --- 동일 높이(y-밴드) 필터 ---
            band_ok = False
            if self.same_y_enable:
                for y_c in self.same_y_targets:
                    if abs(py - y_c) <= self.same_y_tol:
                        # y-밴드 통과 + (직사각형 사용 중이면 in_rect도 만족해야)
                        if (not self.rect_mode_enable) or in_rect:
                            band_ok = True
                            break
                if band_ok:
                    frame_band_points.append((px, py))

            # --- 클러스터링 판정 ---
            # 직사각형 모드: rect_ray_ok일 때만 클러스터 카운트
            # FOV 모드: fov_ok 일 때만 클러스터 카운트
            allow_cluster = rect_ray_ok if self.rect_mode_enable else fov_ok
            if allow_cluster:
                cluster += 1
            else:
                cluster = 0

            if cluster >= self.cluster_min_beams:
                cluster_hit_found = True  # 프레임 내 한 번이라도 만족하면 OK

        # === 프레임 히트 합성(OR): (y-밴드 포인트 수 기준) 또는 (클러스터 기준) ===
        frame_hit = (len(frame_band_points) >= self.min_pts_per_frame) or cluster_hit_found
        self.hit_window.append(1 if frame_hit else 0)

        # === 윈도우 투표 ===
        votes = sum(self.hit_window)
        detected = votes >= self.min_hits_in_window

        # === RViz 마커 갱신 ===
        if self.viz_enable:
            # 직사각형은 런타임 변경 가능성 고려해 갱신
            if self.rect_mode_enable:
                self._publish_rect_marker()

            # y-밴드 포인트 + (옵션) 직사각형 포인트를 함께 표시
            # 필요 시 두 마커로 분리 가능
            viz_points = frame_band_points
            # 직사각형 교차점도 보고 싶다면 아래 주석 해제:
            # viz_points = list(set(frame_band_points + points_in_rect))
            self._publish_hits_marker(viz_points if len(viz_points) > 0 else [])

        # === 정지 신호 출력(디바운스/홀드 포함) ===
        self._publish_stop(detected)

        # === 로그 ===
        shape = "RECT" if self.rect_mode_enable else "FOV"
        band_info = f"band_pts={len(frame_band_points)} (targets={self.same_y_targets}, tol=±{self.same_y_tol:.2f})"
        win_info  = f"window[{len(self.hit_window)}]={list(self.hit_window)} sum={votes}/{self.min_hits_in_window}"
        if detected:
            rospy.logwarn_throttle(0.5, f"[{shape}] STOP=True: {band_info}, cluster_hit={cluster_hit_found}, {win_info}")
        else:
            if self.rect_mode_enable:
                rospy.loginfo_throttle(
                    1.0,
                    f"[{shape}] STOP=False: clear in rect x=[{self.rect_x_offset:.2f},{(self.rect_x_offset+self.rect_length):.2f}] m, "
                    f"y=[{(self.rect_y_center - self.rect_width*0.5):.2f},{(self.rect_y_center + self.rect_width*0.5):.2f}] m, "
                    f"{band_info}, cluster_hit={cluster_hit_found}, {win_info}"
                )
            else:
                rospy.loginfo_throttle(
                    1.0,
                    f"[{shape}] STOP=False: clear in [{rmin:.2f}, {rmax:.2f}] m, FOV({self.forward_angle_range_deg:.1f}°), "
                    f"{band_info}, cluster_hit={cluster_hit_found}, {win_info}"
                )


if __name__ == '__main__':
    try:
        node = LidarObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass