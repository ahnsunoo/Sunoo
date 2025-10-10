import os
import time as ptime
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist  

class DepthEmergencyStopNode:
    def __init__(self):
        # ───────── 파라미터 ─────────
        # 입력 깊이 토픽
        self.depth_topic        = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")

        # 출력 토픽
        self.emergency_topic    = rospy.get_param("~emergency_topic", "/emergency_stop")  # Bool(True=정지)
        self.publish_cmd_vel    = bool(rospy.get_param("~publish_cmd_vel", False))        # 보통 False(통합제어 담당)
        self.cmd_vel_topic      = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # 깊이 스케일/유효범위
        #  - 16UC1(mm) → m 로 환산: depth_m = depth_u16 * depth_scale (보통 0.001)
        #  - 32FC1(m)   → 그대로 사용(무시됨)
        self.depth_scale        = float(rospy.get_param("~depth_scale", 0.001))  # mm→m
        self.max_depth_m        = float(rospy.get_param("~max_depth_m", 10.0))   # 이상값 컷(센서 노이즈)

        # ROI(프레임 하단 비율) & 중앙 폭 비율(필요 시 좁혀서 중앙만 봄)
        self.roi_bottom_ratio   = float(rospy.get_param("~roi_bottom_ratio", 0.35))  # 하단 35%
        self.roi_center_ratio   = float(rospy.get_param("~roi_center_ratio", 1.0))   # 0~1, 1.0=전체폭, 0.6=중앙 60%

        # 장애물 판정 임계
        self.stop_dist_m        = float(rospy.get_param("~stop_dist_m", 1.2))   # 이 거리 이내면 '근거리'
        self.min_near_ratio     = float(rospy.get_param("~min_near_ratio", 0.02)) # ROI 픽셀의 2% 이상이 근거리이면 장애물로 간주
        self.min_near_pixels    = int(rospy.get_param("~min_near_pixels", 200))   # 소규모 노이즈 방지 하한(픽셀 수)

        # 프레임 투표 & 유지/쿨다운
        self.vote_window        = int(rospy.get_param("~vote_window", 3))
        self.vote_threshold     = int(rospy.get_param("~vote_threshold", 2))
        self.stop_hold_sec      = float(rospy.get_param("~stop_hold_sec", 1.5))
        self.cooldown_sec       = float(rospy.get_param("~cooldown_sec", 0.8))

        # 디버그
        self.show_debug         = bool(rospy.get_param("~show_debug", True))
        self.window_name        = rospy.get_param("~window_name", "Depth Emergency Stop Debug")

        # ───────── ROS I/O ─────────
        self.bridge     = CvBridge()
        self.pub_stop   = rospy.Publisher(self.emergency_topic, Bool, queue_size=10)
        self.pub_twist  = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10) if self.publish_cmd_vel else None
        self.sub_depth  = rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1, buff_size=2**24)

        # ───────── 상태 ─────────
        self.votes      = deque(maxlen=self.vote_window)
        self.stop_active = False
        self.stop_until  = 0.0
        self.cooldown_until = 0.0

        rospy.loginfo("[DepthES] Ready. depth_topic=%s, stop_dist=%.2fm, ROI(bottom=%.0f%%, center=%.0f%%)",
                      self.depth_topic, self.stop_dist_m, self.roi_bottom_ratio*100, self.roi_center_ratio*100)

    # 깊이 콜백
    def depth_cb(self, msg: Image):
        # 1) 깊이영상 획득 (16UC1/32FC1 모두 지원)
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("[DepthES] cv_bridge failed: %s", e)
            return

        # 2) 단위를 'm'로 맞추기
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * self.depth_scale      # 16UC1(mm) → m
            invalid = (depth == 0)
        else:
            depth_m = depth.astype(np.float32)                          # 32FC1(m)
            invalid = ~np.isfinite(depth_m)

        H, W = depth_m.shape[:2]
        depth_m[invalid] = np.nan                                       # 무효값 NaN 처리
        depth_m[depth_m > self.max_depth_m] = np.nan                    # 너무 먼 값 컷

        # 3) ROI 선택(하단, 중앙폭)
        top = int(H * (1.0 - self.roi_bottom_ratio))
        if not (0 <= top < H):
            top = max(0, min(H-1, top))
        if self.roi_center_ratio < 1.0:
            cw = int(W * self.roi_center_ratio)
            x0 = (W - cw) // 2
            x1 = x0 + cw
        else:
            x0, x1 = 0, W

        roi = depth_m[top:H, x0:x1]
        roi_valid = np.isfinite(roi) & (roi > 0)

        # 4) 근거리(<= stop_dist_m) 픽셀 비율/갯수 측정
        near = roi_valid & (roi <= self.stop_dist_m)
        near_count = int(np.count_nonzero(near))
        roi_area = roi.shape[0] * roi.shape[1]
        near_ratio = (near_count / float(roi_area)) if roi_area > 0 else 0.0

        # 5) ROI의 최소거리(디버그/안전)
        min_dist = float(np.nanmin(roi)) if np.any(roi_valid) else float('nan')

        # 6) 한 프레임 판정: 면적/비율 둘 다 만족할 때만 True
        intruder = (near_count >= self.min_near_pixels) and (near_ratio >= self.min_near_ratio)

        # 7) 프레임 투표
        self.votes.append(1 if intruder else 0)
        vote_sum = sum(self.votes)
        vote_ok  = (vote_sum >= self.vote_threshold)

        now = ptime.monotonic()

        # 8) 트리거(쿨다운 아님 + 투표충족)
        if (not self.stop_active) and (now >= self.cooldown_until) and vote_ok:
            self.stop_active = True
            self.stop_until  = now + self.stop_hold_sec
            self.publish_stop(True)
            rospy.logwarn("[DepthES] EMERGENCY STOP TRIGGERED (min_dist=%.2fm, near_ratio=%.3f, count=%d)",
                          min_dist, near_ratio, near_count)

        # 9) 유지/해제
        if self.stop_active:
            # (옵션) 정지 명령을 즉시 1회 보내고, 통합제어가 속도 제어
            if self.publish_cmd_vel:
                self.publish_zero_twist()
            # 유지 시간 끝나면 해제 + 쿨다운 시작
            if now >= self.stop_until:
                self.stop_active = False
                self.cooldown_until = now + self.cooldown_sec
                self.publish_stop(False)
                rospy.loginfo("[DepthES] Stop released (cooldown %.1fs)", self.cooldown_sec)

        # 10) 디버그 시각화
        if self.show_debug:
            vis = self.visualize(depth_m, H, W, top, x0, x1, min_dist, near_ratio, intruder, vote_sum)
            try:
                cv2.imshow(self.window_name, vis)
                cv2.waitKey(1)
            except Exception:
                pass  # 헤드리스 환경

    # 디버그 시각화
    def visualize(self, depth_m, H, W, top, x0, x1, min_dist, near_ratio, intruder, vote_sum):
        # 컬러맵(보기 편하게): NaN→0, 범위 클램프
        d = depth_m.copy()
        d[np.isnan(d)] = 0.0
        d = np.clip(d, 0, min(self.max_depth_m, 5.0))     # 0~5m 범위로 색상화
        norm = (d / max(1e-6, d.max())) * 255.0
        vis = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_JET)

        # ROI 박스
        cv2.rectangle(vis, (x0, top), (x1-1, H-1), (0, 255, 255), 2)

        # 텍스트 오버레이
        status = "INTRUDER" if intruder else "clear"
        cv2.putText(vis, f"min={min_dist:.2f}m  near_ratio={near_ratio:.3f}  votes={vote_sum}/{self.vote_window}  {status}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255) if intruder else (0, 255, 0), 2, cv2.LINE_AA)
        return vis

    # Bool 퍼블리시(+ 옵션: 즉시 0속도 1회)
    def publish_stop(self, flag: bool):
        self.pub_stop.publish(Bool(data=flag))
        if flag and self.publish_cmd_vel:
            self.publish_zero_twist()

    # (옵션) 0속도 트위스트 1회
    def publish_zero_twist(self):
        if self.pub_twist is None:
            return
        tw = Twist()  # all zeros
        self.pub_twist.publish(tw)

def main():
    rospy.init_node("depth_emergency_stop_node", anonymous=False)
    try:
        DepthEmergencyStopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()