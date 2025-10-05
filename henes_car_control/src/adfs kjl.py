#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool
import math

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

class LidarObstacleDetector:
    def __init__(self):
        rospy.init_node('lidar_obstacle_detector', anonymous=True)

        # ===== 기본 파라미터(ROS param로 덮어쓰기 가능) =====
        self.obstacle_check_waypoints = rospy.get_param("~check_waypoints", [3, 7, 12])
        self.detection_distance       = float(rospy.get_param("~detection_distance", 5.0))
        self.forward_angle_range_deg  = float(rospy.get_param("~forward_angle_range_deg", 30.0))
        self.front_yaw_offset_deg     = float(rospy.get_param("~front_yaw_offset_deg", 0.0))
        self.near_limit_m             = float(rospy.get_param("~near_limit_m", 0.01))
        self.far_limit_m              = float(rospy.get_param("~far_limit_m", 10.0))
        self.cluster_min_beams        = int(rospy.get_param("~cluster_min_beams", 3))
        self.hold_on_sec              = float(rospy.get_param("~hold_on_sec", 0.3))
        self.hold_off_sec             = float(rospy.get_param("~hold_off_sec", 0.3))

        # 마스크 파라미터: 안전 파싱(문자열/리스트 모두 허용, 잘못된 항목 무시)
        raw_masks = rospy.get_param("~angle_masks_deg", [(90.0, 95.0), (120.0, 180.0)])
        self.angle_masks_deg = self._parse_angle_masks(raw_masks)

        # ===== 내부 상태/IO =====
        self.latest_scan = None
        self.active_zone = False  # WP 콜백으로 on/off
        self.latched_state = False
        self.last_state_change = rospy.Time.now()

        self.obstacle_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=1)
        rospy.Subscriber('/current_waypoint', Int32, self.waypoint_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.loginfo(
            f"LidarObstacleDetector ready:"
            f" WPs={self.obstacle_check_waypoints},"
            f" FOV={self.forward_angle_range_deg:.1f} deg,"
            f" range_win=[{self.near_limit_m:.2f}, {self.far_limit_m:.2f}] m,"
            f" cluster_min={self.cluster_min_beams},"
            f" hold=(+{self.hold_on_sec:.2f}/-{self.hold_off_sec:.2f}) s,"
            f" masks={self.angle_masks_deg}"
        )

    @staticmethod
    def _parse_angle_masks(raw):
        """
        허용 형식:
          - 리스트/튜플: [[-30,30],[150,210]]  또는  [(-30,30),(150,210)]
          - 문자열 CSV: "-30:30,150:210"
          - 문자열 리스트: ["-30~30","150~210"]
        -> 반환: [(a0, a1), ...]  (실수 튜플 목록)
        """
        out = []

        def push(a, b):
            try:
                a = float(a); b = float(b)
                out.append((a, b))
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

    # 웨이포인트 변화 시 검사 on/off 스위치
    def waypoint_callback(self, msg):
        idx = msg.data
        self.active_zone = (idx in self.obstacle_check_waypoints)
        if not self.active_zone:
            self._publish_state(False, force=True)
            rospy.loginfo(f"WP#{idx}: not in check zone → CLEAR(False).")
        else:
            rospy.loginfo(f"WP#{idx}: ACTIVE zone. Scanning each /scan.")

    # 검사 구간이면 매 스캔 프레임마다 검사
    def scan_callback(self, msg):
        self.latest_scan = msg
        if self.active_zone:
            self.check_for_obstacles()

    def _publish_state(self, new_state, force=False):
        """디바운스/홀드 적용 후 퍼블리시"""
        now = rospy.Time.now()

        if force:
            self.latched_state = new_state
            self.last_state_change = now
            self.obstacle_pub.publish(Bool(data=new_state))
            return

        if new_state != self.latched_state:
            elapsed = (now - self.last_state_change).to_sec()
            if new_state is True:
                # False -> True 전환: 최소 미감지 기간 확보
                if elapsed < self.hold_off_sec:
                    return
            else:
                # True -> False 전환: 최소 감지 유지 시간 확보
                if elapsed < self.hold_on_sec:
                    return

            self.latched_state = new_state
            self.last_state_change = now
            self.obstacle_pub.publish(Bool(data=new_state))

    def check_for_obstacles(self):
        scan = self.latest_scan
        if scan is None:
            rospy.logwarn_throttle(1.0, "LiDAR data not available yet.")
            return

        # 유효 거리 범위: 센서 스펙 ∩ 윈도우 ∩ detection_distance
        rmin = max(scan.range_min, self.near_limit_m)
        rmax = min(scan.range_max, self.far_limit_m, self.detection_distance)

        # 차량 기준 FOV (360이면 전방위)
        half = 180.0 if self.forward_angle_range_deg >= 359.9 else self.forward_angle_range_deg / 2.0
        fov_start = -half
        fov_end   =  half

        cluster = 0
        hit_info = None  # (d, i, body_deg)

        # 전체 빔 순회
        for i, d in enumerate(scan.ranges):
            # 일부 드라이버는 미측정에 0.0/inf/Nan을 줌 → 배제
            if not math.isfinite(d) or d <= 0.0 or d < rmin or d > rmax:
                cluster = 0
                continue

            angle_i    = scan.angle_min + i * scan.angle_increment  # [rad], 센서 기준
            sensor_deg = math.degrees(angle_i)
            # 차량 기준: 센서각 - 장착 오프셋(오프셋 양수 = 센서가 좌측으로 돌아가 있음)
            body_deg   = wrap_deg(sensor_deg - self.front_yaw_offset_deg)

            # FOV 체크
            if not in_sector(body_deg, fov_start