#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TL + Cone (Parallel/T) Node (Webcam only)
- HSV 보조 삭제: YOLO만 사용하여 신호등 색상 판별 ('red','yellow','rl','green','unknown')
- 웨이포인트 게이팅 삭제: 항상 추론·퍼블리시
- 기존 True/False로 보내던 신호등 토픽(~stop_topic, 기본 /red_sign)을 String 색상으로 변경
- 라바콘은 항상 ROI 내 감지 여부를 /cone_hit(Bool)로 스트리밍 퍼블리시
- 기존 주차 서비스는 유지하되 웨이포인트 조건 제거(최근 윈도우 집계 기반)
"""

import os, sys, time, collections
from pathlib import Path
from typing import Tuple, Deque, Dict, List
import cv2, numpy as np, torch, rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

# ===========================
# 파라미터(기본값)
# ===========================
DEFAULTS = {
    # ───── 입력(웹캠 전용) ─────
    "cam_index": 0,
    "cap_width": 640,
    "cap_height": 640,
    "cap_fps": 30,

    # ───── YOLOv5 (신호등) ─────
    "yolov5_dir": f"/home/{os.getenv('USER','user')}/catkin_ws/src/yolov5",
    "tl_weights": "tl_colors.pt",   # 신호등 전용 가중치 (red/yellow/rl/green 포함)
    "imgsz": 640,
    "conf_thres": 0.40,
    "iou_thres": 0.45,
    "max_det": 50,
    "tl_yolo_min_conf": 0.35,
    "tl_label_red_csv":    "red",
    "tl_label_green_csv":  "green",
    "tl_label_left_csv":   "rl,left,arrow",
    "tl_label_yellow_csv": "yellow,orange",

    # ───── YOLOv5 (라바콘) ─────
    "cone_yolov5_dir": f"/home/{os.getenv('USER','user')}/catkin_ws/src/yolov5",
    "cone_weights": "cones.pt",
    "cone_imgsz": 640,
    "cone_conf_thres": 0.35,
    "cone_iou_thres": 0.45,
    "cone_max_det": 100,
    "cone_label_csv": "cone",

    # 라바콘 ROI
    "cone_roi_side": "left",   # "left"|"right"|"center"|"full"|"custom"
    "cone_roi_hfrac": 0.5,
    "cone_roi_top_ratio": 0.0,
    "cone_roi_bottom_ratio": 1.0,
    "cone_min_box_area_ratio": 0.0015,

    # 라바콘 서비스 집계 파라미터
    "cone_window_sec": 3.0,
    "cone_min_hits": 3,

    # ───── 디버그/루프 ─────
    "show_debug": True,
    "rate_hz": 30.0,

    # ───── 신호등(TL) 출력/투표 ─────
    "status_topic": "/traffic_light_state",  # 최종 상태 문자열 토픽(색상 그대로)
    "stop_topic":   "/red_sign",             # ★ 변경: Bool→String, 색상 그대로 퍼블리시
    "vote_time_window_sec": 2.0,
    "decide_period_sec":    1.0,
    "history_sec":          10.0,

    # ───── 박스 선택정책 ─────
    "min_box_w": 24, "min_box_h": 24,
    "selection_mode": "hybrid",       # largest|highest_conf|leftmost|rightmost|center_closest|hybrid
    "hybrid_center_weight": 0.2,
    "hybrid_area_weight":   0.8,
}

def P(k: str): return rospy.get_param("~" + k, DEFAULTS[k])

def _parse_labels(csv_text: str) -> List[str]:
    return [t.strip().lower() for t in str(csv_text).split(",") if t.strip()]

class TLAndConeNode:
    def __init__(self):
        # 입력/루프
        self.cam_index  = int(P("cam_index"))
        self.cap_width  = int(P("cap_width"))
        self.cap_height = int(P("cap_height"))
        self.cap_fps    = int(P("cap_fps"))
        self.rate_hz    = float(P("rate_hz"))
        self.show_debug = bool(P("show_debug"))

        # YOLO (신호등)
        self.yolov5_dir   = P("yolov5_dir")
        self.tl_weights   = P("tl_weights")
        self.imgsz        = int(P("imgsz"))
        self.conf_thres   = float(P("conf_thres"))
        self.iou_thres    = float(P("iou_thres"))
        self.max_det      = int(P("max_det"))
        self.tl_yolo_min_conf = float(P("tl_yolo_min_conf"))
        self.label_map_csv = {
            "red":    _parse_labels(P("tl_label_red_csv")),
            "green":  _parse_labels(P("tl_label_green_csv")),
            "rl":     _parse_labels(P("tl_label_left_csv")),
            "yellow": _parse_labels(P("tl_label_yellow_csv")),
        }

        # YOLO (라바콘)
        self.cone_y5_dir     = P("cone_yolov5_dir")
        self.cone_weights    = P("cone_weights")
        self.cone_imgsz      = int(P("cone_imgsz"))
        self.cone_conf_thres = float(P("cone_conf_thres"))
        self.cone_iou_thres  = float(P("cone_iou_thres"))
        self.cone_max_det    = int(P("cone_max_det"))
        self.cone_min_box_area_ratio = float(P("cone_min_box_area_ratio"))
        self.cone_roi_side   = str(P("cone_roi_side")).strip().lower()
        self.cone_roi_hfrac  = float(P("cone_roi_hfrac"))
        self.cone_roi_top_ratio    = float(P("cone_roi_top_ratio"))
        self.cone_roi_bottom_ratio = float(P("cone_roi_bottom_ratio"))

        # 라바콘 집계(서비스용)
        self.cone_window_sec = float(P("cone_window_sec"))
        self.cone_min_hits   = int(P("cone_min_hits"))
        self._hits_parallel: Deque[Tuple[float,int]] = collections.deque(maxlen=4096)
        self._hits_t:        Deque[Tuple[float,int]] = collections.deque(maxlen=4096)

        # 박스 선택/정책
        self.min_box_w  = int(P("min_box_w"))
        self.min_box_h  = int(P("min_box_h"))
        self.selection_mode       = str(P("selection_mode")).strip().lower()
        self.hybrid_center_weight = float(P("hybrid_center_weight"))
        self.hybrid_area_weight   = float(P("hybrid_area_weight"))

        # 퍼블리셔/투표
        self.status_topic = P("status_topic")    # String 색상
        self.stop_topic   = P("stop_topic")      # String 색상(기존 bool 토픽 대체)
        self.vote_time_window_sec = float(P("vote_time_window_sec"))
        self.decide_period_sec    = float(P("decide_period_sec"))
        self.history_sec          = float(P("history_sec"))
        self.pub_state   = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.pub_tlcolor = rospy.Publisher(self.stop_topic,   String, queue_size=10)  # ← 변경됨
        # 단색 Bool 토픽 제거(/tl_red 등)

        # 라바콘 스트림 + 서비스용 주제
        self.pub_cone_hit    = rospy.Publisher("/cone_hit", Bool, queue_size=10)         # 상시 스트리밍
        self.pub_parallel    = rospy.Publisher("/cone_detect_parallel", String, queue_size=1, latch=True)  # 서비스 성공시 "1"
        self.pub_t           = rospy.Publisher("/cone_detect_t",        String, queue_size=1, latch=True)  # 서비스 성공시 "2"
        self.pub_parking_stop= rospy.Publisher("/parking_stop",         Bool,   queue_size=1, latch=True)

        self.state_hist = collections.deque(maxlen=int(max(1000, self.history_sec*self.rate_hz*2)))
        self.last_decide_time = 0.0
        self.last_final_state = "unknown"

        # 서비스(웨이포인트 게이팅 제거)
        self.srv_t        = rospy.Service("/GetParkingPath", Trigger, self._srv_get_t)
        self.srv_parallel = rospy.Service("/GetParallelParkingPath", Trigger, self._srv_get_parallel)

        # YOLO 초기화 & 웹캠
        self._init_yolo_tl()
        self._init_yolo_cone()
        self.cap = None
        self._init_webcam()

        rospy.loginfo("[READY] webcam only | TL/Cone always-on | stop_topic(String)=%s", self.stop_topic)

    # ──────────────────────────────────────────────────
    # 공통/YOLO 초기화
    # ──────────────────────────────────────────────────
    def _common_imports(self):
        from models.common import DetectMultiBackend
        from utils.general import non_max_suppression, scale_boxes
        from utils.augmentations import letterbox
        return DetectMultiBackend, non_max_suppression, scale_boxes, letterbox

    def _init_yolo_tl(self):
        y5_root = Path(self.yolov5_dir).resolve()
        if not (y5_root/"models"/"common.py").exists():
            rospy.logerr("[YOLO-TL] wrong yolov5_dir: %s", str(y5_root))
            raise RuntimeError("yolov5_dir invalid")
        if str(y5_root) not in sys.path: sys.path.insert(0, str(y5_root))
        DetectMultiBackend, nms, scale_boxes, letterbox = self._common_imports()
        self.DetectMultiBackend = DetectMultiBackend
        self.nms = nms; self.scale_boxes = scale_boxes; self.letterbox = letterbox
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.half   = (self.device.type != 'cpu')

        w_tl = Path(self.tl_weights)
        wpath = (Path(self.yolov5_dir)/w_tl) if not w_tl.is_absolute() else w_tl
        if not wpath.exists():
            alt = Path(self.tl_weights)
            if alt.exists(): wpath = alt
            else: raise FileNotFoundError(f"tl_weights not found: {self.tl_weights}")

        self.model = self.DetectMultiBackend(str(wpath), device=self.device, fp16=self.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        try: self.model.warmup(imgsz=(1,3,P("imgsz"),P("imgsz")))
        except Exception: pass
        if isinstance(self.names, dict):
            mx = max(int(k) for k in self.names.keys()); tmp = [""]*(mx+1)
            for k,v in self.names.items(): tmp[int(k)] = v
            self.names = tmp

        # 클래스 이름 → id 매핑(별칭 허용)
        name_to_ids: Dict[str, List[int]] = {}
        for i, n in enumerate(self.names):
            if not n: continue
            name_to_ids.setdefault(n.strip().lower(), []).append(i)
        def ids_for(aliases: List[str]) -> set:
            out=set()
            for a in aliases:
                if a in name_to_ids: out.update(name_to_ids[a]); continue
                for k,v in name_to_ids.items():
                    if a in k: out.update(v)
            return out
        self.tl_ids_map = {
            "red":    ids_for(self.label_map_csv["red"]),
            "green":  ids_for(self.label_map_csv["green"]),
            "rl":     ids_for(self.label_map_csv["rl"]),
            "yellow": ids_for(self.label_map_csv["yellow"]),
        }
        self.tl_all_ids = set().union(*self.tl_ids_map.values())
        rospy.loginfo("[YOLO-TL] class-ids: %s", {k: sorted(list(v)) for k,v in self.tl_ids_map.items()})

    def _init_yolo_cone(self):
        y5_root = Path(self.cone_y5_dir).resolve()
        if not (y5_root/"models"/"common.py").exists():
            rospy.logerr("[YOLO-CONE] wrong cone_yolov5_dir: %s", str(y5_root))
            raise RuntimeError("cone_yolov5_dir invalid")
        if str(y5_root) not in sys.path: sys.path.insert(0, str(y5_root))
        DetectMultiBackend, nms, scale_boxes, letterbox = self._common_imports()
        self.cone_nms = nms; self.cone_scale_boxes = scale_boxes; self.cone_letterbox = letterbox

        w_cone = Path(self.cone_weights)
        wpath = (Path(self.cone_y5_dir)/w_cone) if not w_cone.is_absolute() else w_cone
        if not wpath.exists():
            alt = Path(self.cone_weights)
            if alt.exists(): wpath = alt
            else: raise FileNotFoundError(f"cone_weights not found: {self.cone_weights}")

        self.cone_model = self.DetectMultiBackend(str(wpath), device=self.device, fp16=self.half)
        self.cone_stride, self.cone_names, self.cone_pt = self.cone_model.stride, self.cone_model.names, self.cone_model.pt
        try: self.cone_model.warmup(imgsz=(1,3,P("cone_imgsz"),P("cone_imgsz")))
        except Exception: pass
        if isinstance(self.cone_names, dict):
            mx = max(int(k) for k in self.cone_names.keys()); tmp = [""]*(mx+1)
            for k,v in self.cone_names.items(): tmp[int(k)] = v
            self.cone_names = tmp

        # cone 클래스 id 수집
        cone_label_list = _parse_labels(P("cone_label_csv"))
        name_to_ids: Dict[str, List[int]] = {}
        for i, n in enumerate(self.cone_names):
            if not n: continue
            name_to_ids.setdefault(n.strip().lower(), []).append(i)
        cone_ids = set()
        for alias in cone_label_list:
            if alias in name_to_ids: cone_ids.update(name_to_ids[alias]); continue
            for k,v in name_to_ids.items():
                if alias in k: cone_ids.update(v)
        self.cone_ids = cone_ids
        rospy.loginfo("[YOLO-CONE] ids=%s for labels=%s", sorted(self.cone_ids), cone_label_list)

    def _init_webcam(self):
        backend = cv2.CAP_DSHOW if os.name=='nt' else 0
        self.cap = cv2.VideoCapture(self.cam_index, backend)
        if self.cap_width  > 0: self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.cap_width)
        if self.cap_height > 0: self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_height)
        if self.cap_fps    > 0: self.cap.set(cv2.CAP_PROP_FPS,          self.cap_fps)
        if not self.cap.isOpened(): raise RuntimeError("Webcam open failed (index=%d)" % self.cam_index)

    # ──────────────────────────────────────────────────
    # 루프
    # ──────────────────────────────────────────────────
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            ok, bgr = self.cap.read()
            if not ok or bgr is None:
                rospy.logwarn_throttle(2.0, "[Camera] empty frame"); rate.sleep(); continue
            self._process_and_publish(bgr)
            rate.sleep()

    # ──────────────────────────────────────────────────
    # 메인 처리(항상 동작)
    # ──────────────────────────────────────────────────
    @torch.no_grad()
    def _process_and_publish(self, im0: np.ndarray):
        H, W = im0.shape[:2]
        now = self._now_sec()

        # ─ TL 추론(항상)
        tl_state_instant = self._run_tl_yolo_color(im0, H, W)
        self._tl_append_state(now, tl_state_instant)
        self._tl_trim_history(now)

        publish_now = (now - self.last_decide_time) >= self.decide_period_sec
        if publish_now:
            self.last_final_state = self._tl_time_weighted_winner(now)
            self.last_decide_time = now
            # 색상 문자열 그대로 퍼블리시
            self.pub_state.publish(String(data=self.last_final_state))
            self.pub_tlcolor.publish(String(data=self.last_final_state))  # ← 기존 /red_sign 대체

        # ─ Cone 추론(항상)
        cone_hit, roi_rect = self._run_cone_inference_in_roi(im0, H, W)
        self.pub_cone_hit.publish(Bool(data=bool(cone_hit)))  # 상시 스트림

        # 서비스 집계를 위해 두 버퍼 모두에 동일 누적(모드 선택은 서비스 호출자가 함)
        self._hits_parallel.append((now, 1 if cone_hit else 0))
        self._hits_t.append((now, 1 if cone_hit else 0))

        # ─ 디버그
        if self.show_debug:
            try:
                rx0, ry0, rx1, ry1 = roi_rect
                cv2.rectangle(im0, (rx0, ry0), (rx1, ry1), (0, 255, 255), 2)
                dbg = f"TL={self.last_final_state} cone_hit={cone_hit}"
                cv2.putText(im0, dbg, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
                cv2.imshow("TL + Cone (always-on)", im0); cv2.waitKey(1)
            except Exception:
                pass

    # ──────────────────────────────────────────────────
    # 신호등 — YOLO만 사용
    # ──────────────────────────────────────────────────
    def _run_tl_yolo_color(self, im0, H, W) -> str:
        im = self.letterbox(im0, self.imgsz, stride=self.stride, auto=self.pt)[0]
        im = im.transpose((2,0,1))[::-1]; im = np.ascontiguousarray(im)
        t  = torch.from_numpy(im).to(self.device); t = t.half() if self.half else t.float()
        t /= 255.0
        if t.ndimension()==3: t = t.unsqueeze(0)
        pred = self.model(t, augment=False)
        pred = self.nms(pred, self.conf_thres, self.iou_thres,
                        classes=list(self.tl_all_ids) if self.tl_all_ids else None,
                        max_det=self.max_det)
        cands = []  # (x1,y1,x2,y2, conf, color, dist2, area)
        cx, cy = W*0.5, H*0.5
        for det in pred:
            if not len(det): continue
            det_scaled = det.clone()
            det_scaled[:, :4] = self.scale_boxes(t.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in det_scaled:
                conf = float(conf.item())
                if conf < self.tl_yolo_min_conf: continue
                cls_i = int(cls.item())
                color = None
                for k, idset in self.tl_ids_map.items():
                    if cls_i in idset: color = k; break
                if color is None: continue
                x1,y1,x2,y2 = [int(v.item()) for v in xyxy]
                x1,y1 = max(0,x1), max(0,y1); x2,y2 = min(W-1,x2), min(H-1,y2)
                if x2<=x1 or y2<=y1: continue
                w,h = (x2-x1),(y2-y1)
                if w<self.min_box_w or h<self.min_box_h: continue
                # 가로형 필터(세로형 제거): 종횡비 >= 1.2만 통과
                ar = w/float(h) if h>0 else 0.0
                if ar < 1.2: continue
                bx,by = 0.5*(x1+x2), 0.5*(y1+y2)
                dist2  = (bx-cx)**2 + (by-cy)**2
                area   = w*h
                cands.append((x1,y1,x2,y2, conf, color, dist2, area))
        if not cands: return "unknown"
        x1,y1,x2,y2, conf, color, _, _ = self._select_tl_box_yolo(cands)
        if self.show_debug:
            col_map = {"green": (0,255,0), "rl": (255,255,0), "yellow": (0,165,255), "red": (0,0,255)}
            col = col_map.get(color, (200,200,200))
            cv2.rectangle(im0, (x1,y1), (x2,y2), col, 2)
            cv2.putText(im0, f"{color} {conf:.2f}", (x1, max(0,y1-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
        return color

    def _select_tl_box_yolo(self, cands):
        m = self.selection_mode
        if m == "largest":        return max(cands, key=lambda c: c[7])
        if m == "highest_conf":   return max(cands, key=lambda c: c[4])
        if m == "leftmost":       return min(cands, key=lambda c: 0.5*(c[0]+c[2]))
        if m == "rightmost":      return max(cands, key=lambda c: 0.5*(c[0]+c[2]))
        if m == "center_closest": return min(cands, key=lambda c: c[6])
        if m == "hybrid":
            max_area  = max(x[7] for x in cands) or 1.0
            max_dist2 = max(x[6] for x in cands) or 1.0
            wc, wa = max(0.0, self.hybrid_center_weight), max(0.0, self.hybrid_area_weight)
            scored = []
            for x in cands:
                nd = x[6]/max_dist2; na = x[7]/max_area
                score = wc*nd - wa*na - 0.1*(1.0 - x[4])
                scored.append((score, x))
            return min(scored, key=lambda s: (s[0], -s[1][7]))[1]
        cands.sort(key=lambda c: (c[6], -c[7], -c[4]))
        return cands[0]

    # ──────────────────────────────────────────────────
    # TL 투표·유틸
    # ──────────────────────────────────────────────────
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
        while idx >= 0 and events[idx][0] > cutoff: idx -= 1
        state_at_cut = events[idx][1] if idx >= 0 else events[0][1]
        segs = [(cutoff, state_at_cut)]
        for t, s in events:
            if cutoff <= t <= now_sec: segs.append((t, s))
        last_state = segs[-1][1] if segs else events[-1][1]
        segs.append((now_sec, last_state))
        acc = {"green": 0.0, "rl": 0.0, "red": 0.0, "yellow": 0.0, "unknown": 0.0}
        for i in range(len(segs)-1):
            t0, s0 = segs[i]; t1, _ = segs[i+1]
            acc[s0 if s0 in acc else "unknown"] += max(0.0, t1 - t0)
        prio = {"green": 4, "rl": 3, "red": 2, "yellow": 1, "unknown": 0}
        return max(acc.items(), key=lambda kv: (kv[1], prio.get(kv[0], -1)))[0]

    # ──────────────────────────────────────────────────
    # 라바콘 ROI/추론
    # ──────────────────────────────────────────────────
    def _compute_cone_roi(self, W: int, H: int):
        side = self.cone_roi_side
        w = int(max(0.05, min(1.0, self.cone_roi_hfrac)) * W)
        if side == "left":
            x0, x1 = 0, max(1, w)
        elif side == "right":
            x0, x1 = max(0, W - w), W
        elif side == "center":
            x0 = max(0, (W - w)//2); x1 = min(W, x0 + w)
        elif side == "full":
            x0, x1 = 0, W
        else:
            x0 = max(0, (W - w)//2); x1 = min(W, x0 + w)
        y0 = int(max(0.0, min(1.0, self.cone_roi_top_ratio)) * H)
        y1 = int(max(0.0, min(1.0, self.cone_roi_bottom_ratio)) * H)
        y0, y1 = min(y0, y1-1), max(y0+1, y1)
        return (x0, y0, x1, y1)

    @torch.no_grad()
    def _run_cone_inference_in_roi(self, im0, H, W):
        rx0, ry0, rx1, ry1 = self._compute_cone_roi(W, H)
        roi_rect = (rx0, ry0, rx1, ry1)
        im = self.cone_letterbox(im0, self.cone_imgsz, stride=self.cone_stride, auto=self.cone_pt)[0]
        im = im.transpose((2,0,1))[::-1]; im = np.ascontiguousarray(im)
        t  = torch.from_numpy(im).to(self.device); t = t.half() if self.half else t.float()
        t /= 255.0
        if t.ndimension()==3: t = t.unsqueeze(0)
        pred = self.cone_model(t, augment=False)
        pred = self.cone_nms(pred, self.cone_conf_thres, self.cone_iou_thres,
                             classes=None, max_det=self.cone_max_det)
        cone_hit = False
        for det in pred:
            if not len(det): continue
            det_scaled = det.clone()
            det_scaled[:, :4] = self.cone_scale_boxes(t.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in det_scaled:
                cls_i = int(cls.item())
                if self.cone_ids and (cls_i not in self.cone_ids): continue
                x1,y1,x2,y2 = [int(v.item()) for v in xyxy]
                x1,y1 = max(0,x1), max(0,y1); x2,y2 = min(W-1,x2), min(H-1,y2)
                if x2<=x1 or y2<=y1: continue
                area_ratio = ((x2-x1)*(y2-y1)) / float(W*H)
                if area_ratio < self.cone_min_box_area_ratio: continue
                # ROI 교차 여부
                ix1, iy1 = max(x1, rx0), max(y1, ry0)
                ix2, iy2 = min(x2, rx1), min(y2, ry1)
                if (ix2 > ix1) and (iy2 > iy1):
                    cone_hit = True; break
            if cone_hit: break
        return cone_hit, roi_rect

    # ──────────────────────────────────────────────────
    # 서비스(웨이포인트 게이팅 제거)
    # ──────────────────────────────────────────────────
    def _srv_get_t(self, _req):
        return self._finalize_parking_mode("t")
    def _srv_get_parallel(self, _req):
        return self._finalize_parking_mode("parallel")
    def _finalize_parking_mode(self, mode: str) -> TriggerResponse:
        # 최근 window 내 cone_hit 누적만으로 판단 (웨이포인트 조건 없음)
        start = self._now_sec()
        hits_deque = self._hits_t if mode == "t" else self._hits_parallel
        # start 이전 히트 제거
        while hits_deque and hits_deque[0][0] < start:
            hits_deque.popleft()

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            now = self._now_sec()
            if (now - start) >= self.cone_window_sec:
                break
            rate.sleep()

        now = self._now_sec()
        hits = sum(h for (t,h) in list(hits_deque) if start <= t <= now)
        ok = (hits >= self.cone_min_hits)

        if ok:
            if mode == "t":
                self.pub_t.publish(String(data="2"))
            else:
                self.pub_parallel.publish(String(data="1"))
            self.pub_parking_stop.publish(Bool(data=True))
            code = "2" if mode == "t" else "1"
            return TriggerResponse(success=True, message=f"confirmed {mode}, hits={hits}/{self.cone_min_hits}, code={code}")
        else:
            elapsed = now - start
            return TriggerResponse(success=False, message=f"insufficient hits {hits}/{self.cone_min_hits} in {elapsed:.2f}s")

# ─────────────────────────────────────────────────────
# 엔트리포인트
# ─────────────────────────────────────────────────────
def main():
    rospy.init_node("tl_cone_simple_node", anonymous=False)
    node = None
    try:
        node = TLAndConeNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            if node and getattr(node, "cap", None): node.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
