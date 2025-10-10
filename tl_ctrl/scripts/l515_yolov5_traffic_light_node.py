#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TL+OBS Combo Node (Webcam or ROS Image)
- 한 번의 캡처/추론으로 두 기능을 동시에 수행:
  1) 신호등 판정 (가로 4분할: RED | ORANGE | LEFT | GREEN)
     • 상태 문자열 토픽: /traffic_light_state  ("red|orange|left|green|unknown")
     • 색상별 Bool 토픽: /tl_red, /tl_orange, /tl_left, /tl_green   ← 판정 그대로
     • 정지 게이팅 Bool: /red_sign  ← 웨이포인트 조건을 만족할 때만 True
  2) 장애물 정지  (/emergency_stop)

[웨이포인트 게이팅]
- std_msgs/Int32로 현재 웨이포인트 인덱스를 구독(~waypoint_index_topic)
- 파라미터에 지정된 인덱스에서만 /red_sign=True가 가능
  * ~left_stop_wp_csv:   이 인덱스에서 'left'(좌회전 청색화살표)일 때 정지
  * ~green_stop_wp_csv:  이 인덱스에서 'green'(직진 초록)일 때 정지
- 위 두 집합에 속하지 않는 웨이포인트에서는 /red_sign는 항상 False

[의존]
- ROS Noetic, OpenCV, PyTorch, cv_bridge(ROS 이미지 모드 시), YOLOv5 소스/가중치
"""

import os, sys, time, collections
from pathlib import Path
from typing import Tuple, Set
import cv2, numpy as np, torch, rospy
from std_msgs.msg import String, Bool, Int32

# (ROS 이미지 모드에서만 필요)
try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
except Exception:
    CvBridge, Image = None, None

DEFAULTS = {
    # ───────── 입력/공통 ─────────
    "input_mode": "webcam",                      # "webcam" | "ros_image"
    "cam_index": 0,
    "cap_width": 640, "cap_height": 480, "cap_fps": 30,
    "image_topic": "/camera/color/image_raw",

    # ───────── 웨이포인트 구독 ─────────
    "waypoint_index_topic": "/current_waypoint", # std_msgs/Int32, 현재 웨이포인트 인덱스
    "left_stop_wp_csv":  "",                     # 예: "12,25,40"
    "green_stop_wp_csv": "",                     # 예: "7,19"

    # ───────── YOLOv5 ─────────
    "yolov5_dir": f"/home/{os.getenv('USER','user')}/catkin_ws/src/yolov5",
    "weights": "yolov5m.pt",
    "imgsz": 640, "conf_thres": 0.40, "iou_thres": 0.45, "max_det": 50,

    # ───────── 디버그/루프 ─────────
    "show_debug": True,
    "rate_hz": 30.0,

    # ───────── 신호등(TL) 출력/판정 ─────────
    "status_topic": "/traffic_light_state",      # "red|orange|left|green|unknown"
    "stop_topic":   "/red_sign",                 # 게이팅 결과 Bool (아래 로직 참고)
    "vote_time_window_sec": 2.0,                 # 시간가중 투표창
    "decide_period_sec":    1.0,                 # 최종판정 퍼블리시 주기(초)
    "history_sec":          10.0,

    # 박스 필터/선택정책
    "enable_horizontal_only": True,
    "horiz_ar_min": 1.1,
    "min_box_w": 16, "min_box_h": 16,
    "selection_mode": "hybrid",
    "hybrid_center_weight": 0.2, "hybrid_area_weight": 0.8,

    # ───────── 신호등 색분할(HSV) ─────────
    "edge_crop_ratio": 0.04,
    "morph_kernel": 5,
    "sat_min": 140,
    "val_min": 75,
    "green_h_lo": 80, "green_h_hi": 100,
    "red_h_lo1": 0,  "red_h_hi1": 12,
    "red_h_lo2": 168,"red_h_hi2": 180,
    "orange_h_lo": 12, "orange_h_hi": 32,
    "thr_green_ratio": 0.016,
    "thr_red_ratio":   0.020,
    "thr_orange_ratio":0.018,
    "gamma": 0.78, "clahe_clip": 2.0, "clahe_tile": 8,

    # ───────── TL 판별 방식 선택 ─────────
    # hsv_lr: 좌/우 2분할(왼쪽=red, 오른쪽=green)
    # slots_hsv: 슬롯 기반(노란불 지원, 방향 자동)
    # slots4_hsv: ★가로 4분할(좌→우: red, orange, left, green)
    "tl_method": "slots4_hsv",

    # ───────── 장애물(OBS) 출력/판정 ─────────
    "emergency_topic": "/emergency_stop",
    "stop_classes_csv": "person,car",
    "roi_bottom_ratio": 0.35,
    "roi_center_ratio": 0.8,
    "min_box_area_ratio": 0.1,
    "vote_window": 5, "vote_threshold": 3,
    "stop_hold_sec": 3.0,
    "cooldown_sec": 0.3,
}

def P(k: str):
    return rospy.get_param("~" + k, DEFAULTS[k])

def _parse_csv_to_set(csv_text: str) -> Set[int]:
    s = set()
    for part in str(csv_text).replace(" ", "").split(","):
        if not part: continue
        try:
            s.add(int(part))
        except Exception:
            pass
    return s

class TLObsComboNode:
    """한 번의 입력/추론으로 신호등+장애물 기능을 동시에 수행하는 노드 (웨이포인트 게이팅 포함)"""

    # ──────────────────────────────────────────────────
    # 초기화
    # ──────────────────────────────────────────────────
    def __init__(self):
        # 입력/공통
        self.input_mode   = str(P("input_mode")).strip().lower()
        self.cam_index    = int(P("cam_index"))
        self.cap_width    = int(P("cap_width"))
        self.cap_height   = int(P("cap_height"))
        self.cap_fps      = int(P("cap_fps"))
        self.image_topic  = P("image_topic")
        self.rate_hz      = float(P("rate_hz"))
        self.show_debug   = bool(P("show_debug"))

        # 웨이포인트
        self.wp_topic     = P("waypoint_index_topic")
        self.left_stop_wps  = _parse_csv_to_set(P("left_stop_wp_csv"))
        self.green_stop_wps = _parse_csv_to_set(P("green_stop_wp_csv"))
        self.current_wp   = -1
        self.sub_wp = rospy.Subscriber(self.wp_topic, Int32, self._wp_cb, queue_size=1)

        # YOLO
        self.yolov5_dir   = P("yolov5_dir")
        self.weights      = P("weights")
        self.imgsz        = int(P("imgsz"))
        self.conf_thres   = float(P("conf_thres"))
        self.iou_thres    = float(P("iou_thres"))
        self.max_det      = int(P("max_det"))

        # ───────── 신호등(TL)
        self.status_topic = P("status_topic")
        self.stop_topic   = P("stop_topic")
        self.vote_time_window_sec = float(P("vote_time_window_sec"))
        self.decide_period_sec    = float(P("decide_period_sec"))
        self.history_sec          = float(P("history_sec"))
        self.enable_horizontal_only = bool(P("enable_horizontal_only"))
        self.horiz_ar_min         = float(P("horiz_ar_min"))
        self.min_box_w            = int(P("min_box_w"))
        self.min_box_h            = int(P("min_box_h"))
        self.selection_mode       = str(P("selection_mode")).strip().lower()
        self.hybrid_center_weight = float(P("hybrid_center_weight"))
        self.hybrid_area_weight   = float(P("hybrid_area_weight"))
        self.edge_crop_ratio = float(P("edge_crop_ratio"))
        self.morph_kernel    = int(P("morph_kernel"))
        self.sat_min         = int(P("sat_min"))
        self.val_min         = int(P("val_min"))
        self.green_h_lo      = int(P("green_h_lo")); self.green_h_hi = int(P("green_h_hi"))
        self.red_h_lo1       = int(P("red_h_lo1"));  self.red_h_hi1  = int(P("red_h_hi1"))
        self.red_h_lo2       = int(P("red_h_lo2"));  self.red_h_hi2  = int(P("red_h_hi2"))
        self.orange_h_lo     = int(P("orange_h_lo")); self.orange_h_hi = int(P("orange_h_hi"))
        self.thr_green_ratio = float(P("thr_green_ratio"))
        self.thr_red_ratio   = float(P("thr_red_ratio"))
        self.thr_orange_ratio= float(P("thr_orange_ratio"))
        self.gamma           = float(P("gamma"))
        self.clahe_clip      = float(P("clahe_clip"))
        self.clahe_tile      = int(P("clahe_tile"))

        self.tl_method      = str(P("tl_method")).strip().lower()

        # Publishers & TL state history
        self.pub_state   = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.pub_tlstop  = rospy.Publisher(self.stop_topic,   Bool,   queue_size=10)
        # 색상별 전용 Bool 토픽 (판정 그대로 발행)
        self.pub_red     = rospy.Publisher("/tl_red",    Bool, queue_size=10)
        self.pub_orange  = rospy.Publisher("/tl_orange", Bool, queue_size=10)
        self.pub_left    = rospy.Publisher("/tl_left",   Bool, queue_size=10)
        self.pub_green   = rospy.Publisher("/tl_green",  Bool, queue_size=10)

        self.state_hist = collections.deque(maxlen=int(max(1000, self.history_sec*self.rate_hz*2)))
        self.last_decide_time = 0.0
        self.last_final_state = "unknown"

        # ───────── 장애물(OBS)
        self.emergency_topic   = P("emergency_topic")
        self.stop_classes      = {c.strip().lower() for c in str(P("stop_classes_csv")).split(",") if c.strip()}
        self.roi_bottom_ratio  = float(P("roi_bottom_ratio"))
        self.roi_center_ratio  = float(P("roi_center_ratio"))
        self.min_box_area_ratio= float(P("min_box_area_ratio"))
        self.vote_window       = int(P("vote_window"))
        self.vote_threshold    = int(P("vote_threshold"))
        self.stop_hold_sec     = float(P("stop_hold_sec"))
        self.cooldown_sec      = float(P("cooldown_sec"))

        self.pub_emg = rospy.Publisher(self.emergency_topic, Bool, queue_size=10)
        self.votes = collections.deque(maxlen=self.vote_window)
        self.stop_active = False
        self.stop_until  = 0.0
        self.cooldown_until = 0.0

        # YOLO 초기화
        self._init_yolov5()

        # 입력 초기화
        self.bridge = None
        self.cap    = None
        if self.input_mode == "webcam":
            self._init_webcam()
        elif self.input_mode == "ros_image":
            self._init_ros_image()
            self._timer = rospy.Timer(rospy.Duration(self.decide_period_sec), self._timer_publish)
        else:
            raise RuntimeError("~input_mode must be 'webcam' or 'ros_image'")

        rospy.loginfo(
            "[Combo] ready — TL:(%s,%s), OBS:(%s), input=%s, tl_method=%s, wp_topic=%s, left_wps=%s, green_wps=%s",
            self.status_topic, self.stop_topic, self.emergency_topic,
            self.input_mode, self.tl_method, self.wp_topic,
            sorted(self.left_stop_wps), sorted(self.green_stop_wps)
        )

    # ──────────────────────────────────────────────────
    # 콜백/유틸
    # ──────────────────────────────────────────────────
    def _wp_cb(self, msg: Int32):
        self.current_wp = int(msg.data)

    def _tl_should_stop(self, state: str) -> bool:
        """
        웨이포인트 게이팅:
        - current_wp가 left_stop_wps에 속하고, 상태가 'left'면 True
        - current_wp가 green_stop_wps에 속하고, 상태가 'green'면 True
        - 그 외는 False
        """
        wp = self.current_wp
        if (wp in self.left_stop_wps)  and (state == "left"):
            return True
        if (wp in self.green_stop_wps) and (state == "green"):
            return True
        return False

    # ──────────────────────────────────────────────────
    # YOLO 초기화
    # ──────────────────────────────────────────────────
    def _init_yolov5(self):
        y5_root = Path(self.yolov5_dir).resolve()
        if not (y5_root/"models"/"common.py").exists() or not (y5_root/"utils"/"general.py").exists():
            rospy.logerr("[YOLO] wrong yolov5_dir: %s", str(y5_root))
            raise RuntimeError("yolov5_dir must contain models/common.py and utils/general.py")
        if str(y5_root) not in sys.path:
            sys.path.insert(0, str(y5_root))

        from models.common import DetectMultiBackend
        from utils.general import non_max_suppression, scale_boxes
        from utils.augmentations import letterbox
        self.DetectMultiBackend = DetectMultiBackend
        self.nms         = non_max_suppression
        self.scale_boxes = scale_boxes
        self.letterbox   = letterbox

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.half   = (self.device.type != 'cpu')
        wpath = (y5_root/self.weights) if (y5_root/self.weights).exists() else Path(self.weights)
        self.model = self.DetectMultiBackend(str(wpath), device=self.device, fp16=self.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        try:
            self.model.warmup(imgsz=(1,3,self.imgsz,self.imgsz))
        except Exception:
            pass

        if isinstance(self.names, dict):
            mx = max(int(k) for k in self.names.keys())
            tmp = [""] * (mx + 1)
            for k, v in self.names.items(): tmp[int(k)] = v
            self.names = tmp

        self.tl_ids = [i for i,n in enumerate(self.names) if n and n.strip().lower()=="traffic light"] or None
        stop_ids_set = {i for i,n in enumerate(self.names) if n and n.strip().lower() in self.stop_classes}
        self.stop_ids = stop_ids_set if stop_ids_set else None
        rospy.loginfo("[YOLO] device=%s fp16=%s tl_ids=%s stop_ids#=%s",
                      self.device, self.half, str(self.tl_ids),
                      (len(self.stop_ids) if self.stop_ids is not None else "ALL"))

    # ──────────────────────────────────────────────────
    # 입력 초기화
    # ──────────────────────────────────────────────────
    def _init_webcam(self):
        backend = cv2.CAP_DSHOW if os.name=='nt' else 0
        self.cap = cv2.VideoCapture(self.cam_index, backend)
        if self.cap_width  > 0: self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.cap_width)
        if self.cap_height > 0: self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_height)
        if self.cap_fps    > 0: self.cap.set(cv2.CAP_PROP_FPS,          self.cap_fps)
        if not self.cap.isOpened():
            raise RuntimeError("Webcam open failed (index=%d)" % self.cam_index)

    def _init_ros_image(self):
        if CvBridge is None or Image is None:
            raise RuntimeError("cv_bridge or sensor_msgs/Image not available")
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber(self.image_topic, Image, self._image_cb,
                                        queue_size=1, buff_size=2**24)

    # ──────────────────────────────────────────────────
    # 루프/콜백
    # ──────────────────────────────────────────────────
    def spin(self):
        if self.input_mode == "ros_image":
            rospy.spin()
            return
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            ok, bgr = self.cap.read()
            if not ok or bgr is None:
                rospy.logwarn_throttle(2.0, "[Camera] empty frame")
                rate.sleep(); continue
            self._process_and_publish(bgr)
            rate.sleep()

    @torch.no_grad()
    def _image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("[cv_bridge] %s", e); return
        self._process_and_publish(bgr, ros_mode=True)

    # ──────────────────────────────────────────────────
    # 한 프레임 처리 → (신호등 + 장애물)
    # ──────────────────────────────────────────────────
    @torch.no_grad()
    def _process_and_publish(self, bgr: np.ndarray, ros_mode: bool=False):
        im0 = bgr.copy()
        H, W = im0.shape[:2]

        # 공통: letterbox → 텐서 → 추론 → NMS
        im = self.letterbox(im0, self.imgsz, stride=self.stride, auto=self.pt)[0]
        im = im.transpose((2,0,1))[::-1]          # HWC BGR → CHW RGB
        im = np.ascontiguousarray(im)
        t  = torch.from_numpy(im).to(self.device)
        t  = t.half() if self.half else t.float()
        t /= 255.0
        if t.ndimension()==3: t = t.unsqueeze(0)
        pred = self.model(t, augment=False)
        pred = self.nms(pred, self.conf_thres, self.iou_thres, classes=None, max_det=self.max_det)

        # 좌표 스케일링
        scaled_dets = []
        for det in pred:
            if len(det):
                det_scaled = det.clone()
                det_scaled[:, :4] = self.scale_boxes(t.shape[2:], det[:, :4], im0.shape).round()
                scaled_dets.append(det_scaled)
            else:
                scaled_dets.append(det)

        # ───────── 신호등 파트 ─────────
        tl_candidates = []  # (x1,y1,x2,y2, conf, cls, dist2, area)
        cx, cy = W*0.5, H*0.5
        for det in scaled_dets:
            if len(det):
                for *xyxy, conf, cls in det:
                    cls_i = int(cls.item())
                    if (self.tl_ids is not None) and (cls_i not in self.tl_ids):
                        continue
                    x1,y1,x2,y2 = [int(v.item()) for v in xyxy]
                    x1,y1 = max(0,x1), max(0,y1)
                    x2,y2 = min(W-1,x2), min(H-1,y2)
                    if x2<=x1 or y2<=y1:   continue
                    w,h = (x2-x1),(y2-y1)
                    if w<self.min_box_w or h<self.min_box_h: continue
                    if self.enable_horizontal_only:
                        ar = w/float(h) if h>0 else 0.0
                        if ar < self.horiz_ar_min: continue
                    bx,by = 0.5*(x1+x2), 0.5*(y1+y2)
                    dist2 = (bx-cx)**2 + (by-cy)**2
                    area  = w*h
                    tl_candidates.append((x1,y1,x2,y2,float(conf.item()),cls_i,dist2,area))

        tl_state_instant = "unknown"
        if tl_candidates:
            x1,y1,x2,y2,conf,cls,_,_ = self._select_tl_box(tl_candidates)
            crop = im0[y1:y2, x1:x2]

            if   self.tl_method == "slots4_hsv":
                tl_state_instant = self._classify_slots4_hsv(crop)
            elif self.tl_method == "hsv_lr":
                tl_state_instant = self._classify_lr_hsv(crop)
            elif self.tl_method == "slots_hsv":
                tl_state_instant = self._classify_slots_hsv(crop)
            else:
                tl_state_instant = self._classify_slots4_hsv(crop)

            if self.show_debug:
                color_map = {
                    "green": (0,255,0),
                    "left":  (255,255,0),
                    "orange":(0,165,255),
                    "red":   (0,0,255),
                    "unknown": (200,200,200)
                }
                col = color_map.get(tl_state_instant, (255,255,255))
                name = self.names[cls] if cls < len(self.names) else "tl"
                cv2.rectangle(im0, (x1,y1), (x2,y2), col, 2)
                cv2.putText(im0, f"{name} {conf:.2f} {tl_state_instant}", (x1,max(0,y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)

        # 시간가중 투표 → 최종 상태 결정
        now = self._now_sec()
        self._tl_append_state(now, tl_state_instant)
        self._tl_trim_history(now)
        publish_now = False
        if (now - self.last_decide_time) >= self.decide_period_sec:
            self.last_final_state = self._tl_time_weighted_winner(now)
            self.last_decide_time = now
            publish_now = True

        if publish_now:
            # 최종 상태 문자열
            self.pub_state.publish(String(data=self.last_final_state))
            # 게이팅된 정지 Bool
            tl_stop_bool = self._tl_should_stop(self.last_final_state)
            self.pub_tlstop.publish(Bool(data=tl_stop_bool))
            # 색상별 전용 Bool (판정 그대로, 게이팅 없음)
            st = self.last_final_state
            self.pub_red.publish(Bool(data=(st == "red")))
            self.pub_orange.publish(Bool(data=(st == "orange")))
            self.pub_left.publish(Bool(data=(st == "left")))
            self.pub_green.publish(Bool(data=(st == "green")))

        # ───────── 장애물 파트 ─────────
        intruder = False
        top = int(H * (1.0 - self.roi_bottom_ratio)); top = max(0, min(H-1, top))
        if self.roi_center_ratio < 1.0:
            cw = int(W * self.roi_center_ratio)
            x0 = max(0, (W - cw)//2); x1 = min(W, x0 + cw)
        else:
            x0, x1 = 0, W

        for det in scaled_dets:
            if len(det):
                for *xyxy, conf, cls in det:
                    cls_i = int(cls.item())
                    if (self.stop_ids is not None) and (cls_i not in self.stop_ids):
                        continue
                    x1b,y1b,x2b,y2b = [int(v.item()) for v in xyxy]
                    x1b,y1b = max(0,x1b), max(0,y1b)
                    x2b,y2b = min(W-1,x2b), min(H-1,y2b)
                    if x2b<=x1b or y2b<=y1b: continue
                    area_ratio = ((x2b-x1b)*(y2b-y1b)) / float(W*H)
                    if area_ratio < self.min_box_area_ratio:
                        continue
                    ix1, iy1 = max(x1b,x0), max(y1b,top)
                    ix2, iy2 = min(x2b,x1), min(y2b,H)
                    if (ix2>ix1) and (iy2>iy1):
                        intruder = True

        self.votes.append(1 if intruder else 0)
        vote_sum = sum(self.votes)
        vote_ok  = (vote_sum >= self.vote_threshold)
        nowm = time.monotonic()

        if (not self.stop_active) and (nowm >= self.cooldown_until) and vote_ok:
            self.stop_active = True
            self.stop_until  = nowm + self.stop_hold_sec
            self.pub_emg.publish(Bool(data=True))
            rospy.logwarn("[OBS] EMERGENCY STOP TRIGGERED (%d/%d)", vote_sum, self.vote_window)

        if self.stop_active and nowm >= self.stop_until:
            self.stop_active = False
            self.cooldown_until = nowm + self.cooldown_sec
            self.pub_emg.publish(Bool(data=False))
            rospy.loginfo("[OBS] Stop released (cooldown %.1fs)", self.cooldown_sec)

        if self.show_debug:
            try:
                cv2.rectangle(im0, (x0, top), (x1-1, H-1), (0, 255, 255), 2)
                tlstop_dbg = self._tl_should_stop(self.last_final_state)
                status = f"WP={self.current_wp} TL={self.last_final_state} TLSTOP={tlstop_dbg}  OBS={'INTRUDER' if intruder else 'clear'} votes={vote_sum}/{self.vote_window}"
                color  = (0,0,255) if intruder else (0,255,0)
                cv2.putText(im0, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
                cv2.imshow("TL+OBS Combo", im0); cv2.waitKey(1)
            except Exception:
                pass

    # ──────────────────────────────────────────────────
    # 신호등 보조/판별
    # ──────────────────────────────────────────────────
    def _select_tl_box(self, cands):
        m = self.selection_mode
        if m == "largest":          return max(cands, key=lambda c: c[7])
        if m == "highest_conf":     return max(cands, key=lambda c: c[4])
        if m == "leftmost":         return min(cands, key=lambda c: 0.5*(c[0]+c[2]))
        if m == "rightmost":        return max(cands, key=lambda c: 0.5*(c[0]+c[2]))
        if m == "center_closest":   return min(cands, key=lambda c: c[6])
        if m == "hybrid":
            max_area  = max(x[7] for x in cands) or 1.0
            max_dist2 = max(x[6] for x in cands) or 1.0
            wc, wa = max(0.0, self.hybrid_center_weight), max(0.0, self.hybrid_area_weight)
            scored = []
            for x in cands:
                nd = x[6]/max_dist2; na = x[7]/max_area
                score = wc*nd - wa*na
                scored.append((score, x))
            return min(scored, key=lambda s: (s[0], -s[1][7]))[1]
        cands.sort(key=lambda c: (c[6], -c[7]))
        return cands[0]

    # 2분할 HSV (좌=red/orange, 우=green) — 폴백용
    def _classify_lr_hsv(self, bgr: np.ndarray) -> str:
        if bgr is None or bgr.size == 0:
            return "unknown"
        bgr = self._preprocess_luma(bgr)
        left, right = self._split_left_right(bgr)
        g_ratio = self._ratio_green(right)
        r_ratio = self._ratio_red_or_orange(left)
        if g_ratio >= self.thr_green_ratio:
            return "green"
        if r_ratio >= min(self.thr_red_ratio, self.thr_orange_ratio):
            return "red"
        return "unknown"

    def _split_left_right(self, bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        H, W = bgr.shape[:2]
        if H < 4 or W < 4:
            return bgr, bgr
        pad = int(self.edge_crop_ratio * W)
        x0, x1 = max(0, pad), min(W - pad, W)
        roi = bgr[:, x0:x1]
        mid = roi.shape[1] // 2
        return roi[:, :mid], roi[:, mid:]

    def _post_mask(self, mask: np.ndarray) -> np.ndarray:
        if self.morph_kernel and self.morph_kernel >= 2:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        return mask

    def _ratio_green(self, bgr: np.ndarray) -> float:
        if bgr is None or bgr.size == 0: return 0.0
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        Hc, S, V = cv2.split(hsv)
        base = (S >= self.sat_min) & (V >= self.val_min)
        gmask = base & (Hc >= self.green_h_lo) & (Hc <= self.green_h_hi)
        gmask = self._post_mask(gmask.astype(np.uint8))
        return float(np.count_nonzero(gmask)) / max(1.0, float(Hc.size))

    def _ratio_red(self, bgr: np.ndarray) -> float:
        if bgr is None or bgr.size == 0: return 0.0
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        Hc, S, V = cv2.split(hsv)
        base = (S >= self.sat_min) & (V >= self.val_min)
        r1 = base & (Hc >= self.red_h_lo1) & (Hc <= self.red_h_hi1)
        r2 = base & (Hc >= self.red_h_lo2) & (Hc <= self.red_h_hi2)
        mask = (r1 | r2).astype(np.uint8)
        mask = self._post_mask(mask)
        return float(np.count_nonzero(mask)) / max(1.0, float(Hc.size))

    def _ratio_orange(self, bgr: np.ndarray) -> float:
        if bgr is None or bgr.size == 0: return 0.0
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        Hc, S, V = cv2.split(hsv)
        base = (S >= self.sat_min) & (V >= self.val_min)
        mask = base & (Hc >= self.orange_h_lo) & (Hc <= self.orange_h_hi)
        mask = self._post_mask(mask.astype(np.uint8))
        return float(np.count_nonzero(mask)) / max(1.0, float(Hc.size))

    def _ratio_red_or_orange(self, bgr: np.ndarray) -> float:
        if bgr is None or bgr.size == 0: return 0.0
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        Hc, S, V = cv2.split(hsv)
        base = (S >= self.sat_min) & (V >= self.val_min)
        r1 = base & (Hc >= self.red_h_lo1) & (Hc <= self.red_h_hi1)
        r2 = base & (Hc >= self.red_h_lo2) & (Hc <= self.red_h_hi2)
        o  = base & (Hc >= self.orange_h_lo) & (Hc <= self.orange_h_hi)
        mask = (r1 | r2 | o).astype(np.uint8)
        mask = self._post_mask(mask)
        return float(np.count_nonzero(mask)) / max(1.0, float(Hc.size))

    # 슬롯-HSV (노란불 포함) — 폴백용/호환
    def _classify_slots_hsv(self, bgr: np.ndarray) -> str:
        if bgr is None or bgr.size == 0:
            return "unknown"
        bgr = self._preprocess_luma(bgr)
        H, W = bgr.shape[:2]
        ar = W / max(1, float(H))
        if ar > 1.2:   orient = "h"
        elif ar < 0.8: orient = "v"
        else:          orient = "auto"

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        Hc, S, V = cv2.split(hsv)
        base = (S >= self.sat_min) & (V >= max(self.val_min, 120))
        mask = base.astype(np.uint8)
        if self.morph_kernel and self.morph_kernel >= 2:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)

        def split_slots(img, slots, axis):
            spans = np.linspace(0, img.shape[1-axis], slots+1, dtype=int)
            rois = []
            for i in range(slots):
                if axis == 1:
                    x0, x1 = spans[i], spans[i+1]
                    rois.append((slice(0, H), slice(x0, x1)))
                else:
                    y0, y1 = spans[i], spans[i+1]
                    rois.append((slice(y0, y1), slice(0, W)))
            return rois

        def is_green(h):  return 75 <= h <= 100
        def is_yellow(h): return 18 <= h <= 35
        def is_red(h):    return (0 <= h <= 12) or (168 <= h <= 180)

        def score_and_color(rois):
            best = ("unknown", -1, None)
            for r in rois:
                m = mask[r]
                if np.count_nonzero(m) < 10:
                    continue
                sc = float(np.sum(V[r] * (m>0)))
                if sc <= best[1]:
                    continue
                Hslot = Hc[r][m>0].astype(np.float32)
                mean_h = float(np.mean(Hslot)) if Hslot.size else -1.0
                col = "unknown"
                if mean_h >= 0:
                    if is_green(mean_h):   col = "green"
                    elif is_yellow(mean_h):col = "orange"
                    elif is_red(mean_h):   col = "red"
                best = (col, sc, r)
            return best[0]

        if orient == "h":
            rois = split_slots(bgr, 2, axis=1)
            return score_and_color(rois)
        elif orient == "v":
            rois = split_slots(bgr, 3, axis=0)
            return score_and_color(rois)
        else:
            col_h = score_and_color(split_slots(bgr, 2, axis=1))
            col_v = score_and_color(split_slots(bgr, 3, axis=0))
            for c in ("green","red","orange"):
                if col_h == c or col_v == c:
                    return c
            return "unknown"

    # 가로 4분할: 좌→우 = red, orange, left, green
    def _classify_slots4_hsv(self, bgr: np.ndarray) -> str:
        if bgr is None or bgr.size == 0:
            return "unknown"
        H, W = bgr.shape[:2]
        if H < 4 or W < 8:
            return "unknown"

        bgr = self._preprocess_luma(bgr)
        pad = int(self.edge_crop_ratio * W)
        x0, x1 = max(0, pad), min(W - pad, W)
        roi = bgr[:, x0:x1]
        if roi.shape[1] < 4:
            return "unknown"

        sw = roi.shape[1] // 4
        slots = [roi[:, i*sw:(i+1)*sw] if i<3 else roi[:, i*sw:] for i in range(4)]

        def green_ratio_and_score(img):
            if img.size==0: return 0.0, 0.0
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            Hc,S,V = cv2.split(hsv)
            base = (S >= self.sat_min) & (V >= self.val_min)
            mask = base & (Hc >= self.green_h_lo) & (Hc <= self.green_h_hi)
            mask = self._post_mask(mask.astype(np.uint8))
            ratio = float(np.count_nonzero(mask)) / max(1.0, float(Hc.size))
            score = float(np.sum(V[mask>0]))
            return ratio, score

        def red_ratio_and_score(img):
            if img.size==0: return 0.0, 0.0
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            Hc,S,V = cv2.split(hsv)
            base = (S >= self.sat_min) & (V >= self.val_min)
            r1 = base & (Hc >= self.red_h_lo1) & (Hc <= self.red_h_hi1)
            r2 = base & (Hc >= self.red_h_lo2) & (Hc <= self.red_h_hi2)
            mask = (r1 | r2).astype(np.uint8)
            mask = self._post_mask(mask)
            ratio = float(np.count_nonzero(mask)) / max(1.0, float(Hc.size))
            score = float(np.sum(V[mask>0]))
            return ratio, score

        def orange_ratio_and_score(img):
            if img.size==0: return 0.0, 0.0
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            Hc,S,V = cv2.split(hsv)
            base = (S >= self.sat_min) & (V >= self.val_min)
            mask = base & (Hc >= self.orange_h_lo) & (Hc <= self.orange_h_hi)
            mask = self._post_mask(mask.astype(np.uint8))
            ratio = float(np.count_nonzero(mask)) / max(1.0, float(Hc.size))
            score = float(np.sum(V[mask>0]))
            return ratio, score

        r1, s1 = red_ratio_and_score(slots[0])      # RED
        r2, s2 = orange_ratio_and_score(slots[1])   # ORANGE
        r3, s3 = green_ratio_and_score(slots[2])    # LEFT (green slot)
        r4, s4 = green_ratio_and_score(slots[3])    # GREEN

        candidates = []
        if r1 >= self.thr_red_ratio:     candidates.append(("red",    s1))
        if r2 >= self.thr_orange_ratio:  candidates.append(("orange", s2))
        if r3 >= self.thr_green_ratio:   candidates.append(("left",   s3))
        if r4 >= self.thr_green_ratio:   candidates.append(("green",  s4))

        if not candidates:
            return "unknown"
        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates[0][0]

    # ──────────────────────────────────────────────────
    # 공통 전처리/시간 누적
    # ──────────────────────────────────────────────────
    def _preprocess_luma(self, bgr: np.ndarray) -> np.ndarray:
        if bgr is None or bgr.size == 0:
            return bgr
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(self.clahe_tile, self.clahe_tile))
        v2 = clahe.apply(v)
        inv = 1.0 / max(self.gamma, 1e-6)
        lut = np.array([((i/255.0)**inv)*255.0 for i in range(256)], dtype=np.float32)
        v3 = cv2.LUT(v2, np.clip(lut, 0, 255).astype("uint8"))
        return cv2.cvtColor(cv2.merge([h, s, v3]), cv2.COLOR_HSV2BGR)

    def _now_sec(self) -> float:
        t = rospy.get_time()
        return t if t > 0 else time.time()

    def _tl_append_state(self, t_sec: float, state: str):
        self.state_hist.append((t_sec, state))

    def _tl_trim_history(self, now_sec: float):
        cutoff = now_sec - max(self.history_sec, self.vote_time_window_sec * 3.0)
        while self.state_hist and self.state_hist[0][0] < cutoff:
            self.state_hist.popleft()

    def _tl_time_weighted_winner(self, now_sec: float) -> str:
        if not self.state_hist: return "unknown"
        window = self.vote_time_window_sec
        cutoff = now_sec - window
        events = list(self.state_hist)
        idx = len(events) - 1
        while idx >= 0 and events[idx][0] > cutoff:
            idx -= 1
        state_at_cut = events[idx][1] if idx >= 0 else events[0][1]
        segs = [(cutoff, state_at_cut)]
        for t, s in events:
            if cutoff <= t <= now_sec:
                segs.append((t, s))
        last_state = segs[-1][1] if segs else events[-1][1]
        segs.append((now_sec, last_state))
        acc = {"green": 0.0, "left": 0.0, "red": 0.0, "orange": 0.0, "unknown": 0.0}
        for i in range(len(segs)-1):
            t0, s0 = segs[i]; t1, _ = segs[i+1]
            acc[s0 if s0 in acc else "unknown"] += max(0.0, t1 - t0)
        prio = {"green": 4, "left": 3, "red": 2, "orange": 1, "unknown": 0}
        return max(acc.items(), key=lambda kv: (kv[1], prio.get(kv[0], -1)))[0]

    # ──────────────────────────────────────────────────
    # 타이머: ros_image 모드 TL 퍼블리시
    # ──────────────────────────────────────────────────
    def _timer_publish(self, _evt):
        now = self._now_sec()
        self.last_final_state = self._tl_time_weighted_winner(now)
        self.last_decide_time = now
        # 상태 문자열
        self.pub_state.publish(String(data=self.last_final_state))
        # 게이팅된 정지 Bool
        tl_stop_bool = self._tl_should_stop(self.last_final_state)
        self.pub_tlstop.publish(Bool(data=tl_stop_bool))
        # 색상별 전용 Bool (게이팅 없음)
        st = self.last_final_state
        self.pub_red.publish(Bool(data=(st == "red")))
        self.pub_orange.publish(Bool(data=(st == "orange")))
        self.pub_left.publish(Bool(data=(st == "left")))
        self.pub_green.publish(Bool(data=(st == "green")))

# ─────────────────────────────────────────────────────
# 엔트리포인트
# ─────────────────────────────────────────────────────
def main():
    rospy.init_node("tl_obs_combo_node", anonymous=False)
    node = None
    try:
        node = TLObsComboNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            if node and node.cap: node.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
