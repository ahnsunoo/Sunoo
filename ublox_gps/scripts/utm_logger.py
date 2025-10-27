#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import threading
from datetime import datetime

import rospy
from geometry_msgs.msg import Pose2D

def euclid(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)

class UTMLogger:
    def __init__(self):
        # ── 파라미터 ─────────────────────────────────────────────
        default_dir = os.path.expanduser("~/utm_logs")
        default_name = f"utm_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.output_dir  = rospy.get_param("~output_dir", default_dir)
        self.output_file = rospy.get_param("~output_file", default_name)
        self.topic       = rospy.get_param("~topic", "/gps/utm_pos1")
        self.min_distance = float(rospy.get_param("~min_distance", 0.0))  # m; 0이면 모든 샘플 저장
        self.flush_every  = int(rospy.get_param("~flush_every", 50))      # N개마다 디스크로 flush

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, self.output_file)

        self.buffer = []          # [(x, y), ...]
        self.last_xy = None
        self.count_total = 0
        self.lock = threading.Lock()

        # 파일 생성 & 헤더 기록
        self._init_csv()

        # 구독 시작
        self.sub = rospy.Subscriber(self.topic, Pose2D, self.cb, queue_size=50)

        rospy.loginfo(f"[UTMLogger] Topic: {self.topic}")
        rospy.loginfo(f"[UTMLogger] Saving to: {self.csv_path}")
        rospy.on_shutdown(self.on_shutdown)

    def _init_csv(self):
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(["x", "y"])  # 헤더

    def cb(self, msg: Pose2D):
        x, y = msg.x, msg.y
        with self.lock:
            if self.last_xy is not None and self.min_distance > 0.0:
                if euclid((x, y), self.last_xy) < self.min_distance:
                    return  # 너무 가까우면 스킵
            self.buffer.append((x, y))
            self.last_xy = (x, y)
            self.count_total += 1

            # 주기적으로 디스크 반영
            if len(self.buffer) >= self.flush_every:
                self._flush()

    def _flush(self):
        if not self.buffer:
            return
        with open(self.csv_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerows(self.buffer)
        flushed = len(self.buffer)
        self.buffer.clear()
        rospy.loginfo(f"[UTMLogger] Flushed {flushed} rows (total={self.count_total}).")

    def on_shutdown(self):
        # 남은 데이터 모두 저장
        with self.lock:
            self._flush()
        rospy.loginfo(f"[UTMLogger] Done. File saved at: {self.csv_path}")

def main():
    rospy.init_node("utm_logger", anonymous=True)
    UTMLogger()
    rospy.spin()

if __name__ == "__main__":
    main()
