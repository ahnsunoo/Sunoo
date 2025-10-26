#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Parking Vision (Service Server) — Webcam or ROS Image (Outdoor-Optimized)
- /parking/decide (std_srvs/Trigger): N프레임 수집 → 라바콘 좌/우 판단 → 빈쪽(LEFT/RIGHT/UNKNOWN) 반환
- 입력 소스:
  1) webcam: OpenCV VideoCapture(index)
  2) ros_image: /camera/image_raw 구독(cv_bridge)

실외 최적화 포인트:
- 오렌지 듀얼 Hue 범위(랩어라운드 포함)
- 높은 S(채도) 하한 + 낮은 V(밝기) 하한 (그늘 허용)
- 감마 보정 + CLAHE로 역광/그늘 안정화
- Morphology Open+Close로 콘 덩어리화
- ROI 비례 min_area (해상도 바뀌어도 자동 스케일)

디버그:
- show_debug=True면 항상 카메라 창 표시(ROI 박스/좌우 마스크)
"""

import os
import cv2
import time
import numpy as np
import rospy
from std_srvs.srv import Trigger, TriggerResponse

# ROS 이미지 모드에서 필요 모듈 (cv_bridge, sensor_msgs)
try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
except Exception:
    CvBridge, Image = None, None


class ParkingVisionSrvNode:
    """주차 판단 서비스 서버 노드"""

    def __init__(self):
        # ───── 입력 파라미터 ─────
        self.input_mode  = rospy.get_param('~input_mode', 'ros_image').strip().lower()  # 'webcam' | 'ros_image'
        self.cam_index   = int(rospy.get_param('~cam_index', 0))
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.cap_width   = int(rospy.get_param('~cap_width', 640))
        self.cap_height  = int(rospy.get_param('~cap_height', 480))
        self.cap_fps     = int(rospy.get_param('~cap_fps', 30))
        self.show_debug  = bool(rospy.get_param('~show_debug', False))

        # ───── 판단/ROI/투표 파라미터 ─────
        self.frames   = int(rospy.get_param('~frames', 30))
        self.timeout  = float(rospy.get_param('~timeout', 5.0))
        self.y0_ratio = float(rospy.get_param('~y0_ratio', 0.50))   # 실외: 하단 50%만 사용
        self.margin   = float(rospy.get_param('~margin', 1.25))

        # ROI 비례 min_area: -1이면 자동(ROI 픽셀 * 비율)
        self.min_area = int(rospy.get_param('~min_area', -1))
        self.min_area_ratio = float(rospy.get_param('~min_area_ratio', 0.012))  # 실외 권장 1.2%

        # ───── 영상 보정 파라미터(실외) ─────
        self.use_gamma = bool(rospy.get_param('~use_gamma', True))
        self.gamma = float(rospy.get_param('~gamma', 0.90))  # <1.0 → 약간 밝게, 역광 완화
        self.use_clahe = bool(rospy.get_param('~use_clahe', True))
        self.clahe_clip = float(rospy.get_param('~clahe_clip', 2.0))

        # ───── Morphology ─────
        self.kernel_open  = int(rospy.get_param('~kernel_open', 5))
        self.kernel_close = int(rospy.get_param('~kernel_close', 7))

        # ───── HSV 범위(실외 기본값) ─────
        # 오렌지 듀얼 구간(랩어라운드 포함) + 높은 S 하한, V 하한은 낮게(그늘 허용)
        orange_lo1 = rospy.get_param('~orange_lo1', [8, 120, 60])   # H≈8~22
        orange_hi1 = rospy.get_param('~orange_hi1', [22, 255, 255])
        orange_lo2 = rospy.get_param('~orange_lo2', [0, 120, 60])   # H≈0~5
        orange_hi2 = rospy.get_param('~orange_hi2', [5, 255, 255])

        # 파랑(실외용 S 하한↑, V 하한↓)
        blue_lo = rospy.get_param('~blue_lo', [100, 120, 50])
        blue_hi = rospy.get_param('~blue_hi', [130, 255, 255])

        self.orange_lo1 = np.array(orange_lo1, dtype=np.uint8)
        self.orange_hi1 = np.array(orange_hi1, dtype=np.uint8)
        self.orange_lo2 = np.array(orange_lo2, dtype=np.uint8)
        self.orange_hi2 = np.array(orange_hi2, dtype=np.uint8)
        self.blue_lo    = np.array(blue_lo,    dtype=np.uint8)
        self.blue_hi    = np.array(blue_hi,    dtype=np.uint8)

        # ───── 내부 상태 ─────
        self.bridge = None
        self.last_frame = None
        self.cap = None

        # ───── 입력 초기화 ─────
        if self.input_mode == 'webcam':
            self._init_webcam()
        elif self.input_mode == 'ros_image':
            self._init_ros_image()
        else:
            raise RuntimeError("~input_mode must be 'webcam' or 'ros_image'")

        # ───── 서비스 등록 ─────
        self.srv = rospy.Service('/parking/decide', Trigger, self.handle_decide)
        rospy.loginfo("[parking_vision_srv] ready: /parking/decide  (input=%s, frames=%d, timeout=%.1fs)",
                      self.input_mode, self.frames, self.timeout)

        # ───── 상시 디버그 뷰 ─────
        if self.show_debug:
            self._dbg_timer = rospy.Timer(rospy.Duration(0.05), self._debug_tick)  # ≈20 FPS

    # ========== 카메라 초기화 ==========
    def _init_webcam(self):
        backend = cv2.CAP_DSHOW if os.name == 'nt' else 0
        self.cap = cv2.VideoCapture(self.cam_index, backend)
        if self.cap_width  > 0: self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.cap_width)
        if self.cap_height > 0: self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_height)
        if self.cap_fps    > 0: self.cap.set(cv2.CAP_PROP_FPS,          self.cap_fps)
        if not self.cap.isOpened():
            raise RuntimeError("Webcam open failed (index=%d)" % self.cam_index)
        rospy.loginfo("[camera] webcam opened index=%d (%dx%d@%dfps)", self.cam_index, self.cap_width, self.cap_height, self.cap_fps)

    def _init_ros_image(self):
        if CvBridge is None or Image is None:
            raise RuntimeError("cv_bridge/sensor_msgs not available — install ros-noetic-cv-bridge")
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber(self.image_topic, Image, self._img_cb, queue_size=1, buff_size=2**24)
        rospy.loginfo("[camera] subscribing ROS image: %s", self.image_topic)

    def _img_cb(self, msg: 'Image'):
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn("[cv_bridge] %s", e)

    def _get_frame(self):
        if self.input_mode == 'webcam':
            ok, bgr = self.cap.read()
            return bgr if ok else None
        else:
            return self.last_frame

    # ========== 감마 보정 ==========
    def _gamma_correct(self, bgr, gamma):
        if (gamma is None) or (abs(gamma - 1.0) < 1e-3):
            return bgr
        inv = 1.0 / max(1e-6, gamma)
        table = (np.linspace(0, 1, 256) ** inv * 255.0).astype(np.uint8)
        return cv2.LUT(bgr, table)

    # ========== 서비스 콜백 ==========
    def handle_decide(self, _req):
        votes = []
        last_l, last_r = 0, 0
        t_end = rospy.Time.now() + rospy.Duration(self.timeout)

        while len(votes) < self.frames and rospy.Time.now() < t_end:
            frame = self._get_frame()
            if frame is None:
                rospy.sleep(0.03)
                continue

            side, area_l, area_r = self._decide_cone_side(frame)
            last_l, last_r = area_l, area_r
            if side != 'UNKNOWN':
                votes.append(side)

            if self.show_debug:
                ov = self._make_overlay(frame, area_l, area_r, side)
                try:
                    cv2.imshow('parking_vision_debug', ov)
                    cv2.waitKey(1)
                except Exception:
                    pass

        left_cnt  = votes.count('LEFT')
        right_cnt = votes.count('RIGHT')
        if   left_cnt  > right_cnt: empty = 'RIGHT'
        elif right_cnt > left_cnt:  empty = 'LEFT'
        else:                       empty = 'UNKNOWN'

        ok = (empty != 'UNKNOWN')
        msg = f"{empty} l={last_l} r={last_r} votes(L/R)={left_cnt}/{right_cnt}"
        rospy.loginfo("[parking_vision_srv] %s", msg)
        return TriggerResponse(success=ok, message=msg)

    # ========== 단일 프레임 판정(실외 최적화) ==========
    def _decide_cone_side(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * self.y0_ratio)
        mid = w // 2

        left  = frame[y0:, :mid]
        right = frame[y0:, mid:]

        # 감마 보정 → HSV → CLAHE(V)
        if self.use_gamma:
            left  = self._gamma_correct(left,  self.gamma)
            right = self._gamma_correct(right, self.gamma)

        hsv_l = cv2.cvtColor(left,  cv2.COLOR_BGR2HSV)
        hsv_r = cv2.cvtColor(right, cv2.COLOR_BGR2HSV)

        if self.use_clahe:
            clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(8,8))
            hsv_l[:, :, 2] = clahe.apply(hsv_l[:, :, 2])
            hsv_r[:, :, 2] = clahe.apply(hsv_r[:, :, 2])

        # 오렌지 듀얼 + 파랑
        m_or_l = (cv2.inRange(hsv_l, self.orange_lo1, self.orange_hi1) |
                  cv2.inRange(hsv_l, self.orange_lo2, self.orange_hi2))
        m_or_r = (cv2.inRange(hsv_r, self.orange_lo1, self.orange_hi1) |
                  cv2.inRange(hsv_r, self.orange_lo2, self.orange_hi2))
        m_bl_l = cv2.inRange(hsv_l, self.blue_lo, self.blue_hi)
        m_bl_r = cv2.inRange(hsv_r, self.blue_lo, self.blue_hi)

        m_left_raw  = m_or_l | m_bl_l
        m_right_raw = m_or_r | m_bl_r

        # Morphology: Open → Close
        ko = max(1, self.kernel_open | 1)
        kc = max(1, self.kernel_close | 1)
        k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko))
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kc, kc))
        m_left  = cv2.morphologyEx(cv2.morphologyEx(m_left_raw,  cv2.MORPH_OPEN,  k_open),  cv2.MORPH_CLOSE, k_close)
        m_right = cv2.morphologyEx(cv2.morphologyEx(m_right_raw, cv2.MORPH_OPEN,  k_open),  cv2.MORPH_CLOSE, k_close)

        area_l = int(cv2.countNonZero(m_left))
        area_r = int(cv2.countNonZero(m_right))

        roi_pixels = left.shape[0] * left.shape[1]
        min_area = self.min_area if self.min_area >= 0 else int(roi_pixels * self.min_area_ratio)

        if area_l < min_area and area_r < min_area:
            return 'UNKNOWN', area_l, area_r
        if area_l > area_r * self.margin:
            return 'LEFT', area_l, area_r
        if area_r > area_l * self.margin:
            return 'RIGHT', area_l, area_r
        return 'UNKNOWN', area_l, area_r

    # ========== 오버레이(원본+ROI+면적/비율/판정) ==========
    def _make_overlay(self, frame, area_l, area_r, side):
        h, w = frame.shape[:2]
        y0 = int(h * self.y0_ratio)
        mid = w // 2

        ov = frame.copy()
        cv2.rectangle(ov, (0, y0), (mid-1, h-1), (0,255,0), 2)
        cv2.rectangle(ov, (mid, y0), (w-1, h-1), (255,0,0), 2)

        ratio = float(area_l) / (area_r + 1e-6)
        cv2.putText(ov, f"L:{area_l}", (10, y0-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(ov, f"R:{area_r}", (mid+10, y0-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
        cv2.putText(ov, f"ratio(L/R):{ratio:.2f} side:{side}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        return ov

    # ========== ROI 박스만 시각화(상시 디버그용) ==========
    def _show_roi_debug(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * self.y0_ratio)
        mid = w // 2
        ov = frame.copy()
        cv2.rectangle(ov, (0, y0), (mid-1, h-1), (0,255,0), 2)
        cv2.rectangle(ov, (mid, y0), (w-1, h-1), (255,0,0), 2)
        cv2.putText(ov, "LEFT ROI",  (10, y0-10),     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(ov, "RIGHT ROI", (mid+10, y0-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
        return ov

    # ========== 좌/우 마스크 시각화(상시 디버그용) ==========
    def _make_lr_masks(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * self.y0_ratio)
        mid = w // 2

        left  = frame[y0:, :mid]
        right = frame[y0:, mid:]

        if self.use_gamma:
            left  = self._gamma_correct(left,  self.gamma)
            right = self._gamma_correct(right, self.gamma)

        hsv_l = cv2.cvtColor(left,  cv2.COLOR_BGR2HSV)
        hsv_r = cv2.cvtColor(right, cv2.COLOR_BGR2HSV)

        if self.use_clahe:
            clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(8,8))
            hsv_l[:, :, 2] = clahe.apply(hsv_l[:, :, 2])
            hsv_r[:, :, 2] = clahe.apply(hsv_r[:, :, 2])

        m_or_l = (cv2.inRange(hsv_l, self.orange_lo1, self.orange_hi1) |
                  cv2.inRange(hsv_l, self.orange_lo2, self.orange_hi2))
        m_or_r = (cv2.inRange(hsv_r, self.orange_lo1, self.orange_hi1) |
                  cv2.inRange(hsv_r, self.orange_lo2, self.orange_hi2))
        m_bl_l = cv2.inRange(hsv_l, self.blue_lo, self.blue_hi)
        m_bl_r = cv2.inRange(hsv_r, self.blue_lo, self.blue_hi)

        ko = max(1, self.kernel_open | 1)
        kc = max(1, self.kernel_close | 1)
        k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko))
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kc, kc))
        ml = cv2.morphologyEx(cv2.morphologyEx(m_or_l | m_bl_l, cv2.MORPH_OPEN,  k_open),  cv2.MORPH_CLOSE, k_close)
        mr = cv2.morphologyEx(cv2.morphologyEx(m_or_r | m_bl_r, cv2.MORPH_OPEN,  k_open),  cv2.MORPH_CLOSE, k_close)

        return cv2.cvtColor(ml, cv2.COLOR_GRAY2BGR), cv2.cvtColor(mr, cv2.COLOR_GRAY2BGR)

    # ========== 상시 디버그 갱신 ==========
    def _debug_tick(self, _event):
        frame = self._get_frame()
        if frame is None:
            return
        try:
            roi_vis = self._show_roi_debug(frame)
            cv2.imshow("ParkingVision Camera (ROI)", roi_vis)

            ml_bgr, mr_bgr = self._make_lr_masks(frame)
            cv2.imshow("LEFT mask (orange|blue)",  ml_bgr)
            cv2.imshow("RIGHT mask (orange|blue)", mr_bgr)

            cv2.waitKey(1)
        except Exception:
            pass


def main():
    rospy.init_node('parking_vision_srv')
    node = None
    try:
        node = ParkingVisionSrvNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            if node and node.cap:
                node.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()
