#!/usr/bin/env python3
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--waypoints", required=True, help="웨이포인트 CSV 경로 (열: x,y)")
    ap.add_argument("--track", required=True, help="주행 로그 CSV 경로 (열: time,x,y,yaw,wp_idx,mode,...)")
    ap.add_argument("--out", default="track_plot.png", help="결과 이미지 파일명")
    ap.add_argument("--annotate", action="store_true", help="웨이포인트 인덱스 숫자 표기")
    args = ap.parse_args()

    # CSV 로드
    wps = pd.read_csv(args.waypoints)
    track = pd.read_csv(args.track)

    # 기본 체크
    for col in ["x","y"]:
        if col not in wps.columns:
            raise ValueError(f"waypoints.csv에 '{col}' 열이 필요합니다.")
    for col in ["x","y","yaw","mode"]:
        if col not in track.columns:
            raise ValueError(f"track.csv에 '{col}' 열이 필요합니다. (x,y,yaw,mode)")

    # Figure
    plt.figure(figsize=(10, 8))
    plt.title("Waypoints vs. Vehicle Trajectory")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.4)

    # 1) 웨이포인트 (회색 점 + 시작/종점 강조)
    plt.scatter(wps["x"], wps["y"], s=18, label="Waypoints", alpha=0.8)
    plt.scatter(wps["x"].iloc[0], wps["y"].iloc[0], s=80, marker="o", edgecolors="k", facecolors="none", label="Start (WP0)")
    plt.scatter(wps["x"].iloc[-1], wps["y"].iloc[-1], s=80, marker="s", edgecolors="k", facecolors="none", label=f"Goal (WP{len(wps)-1})")

    # 웨이포인트 인덱스 라벨 옵션
    if args.annotate:
        for i, (x, y) in enumerate(zip(wps["x"], wps["y"])):
            plt.text(x, y, str(i), fontsize=8, ha="center", va="bottom", alpha=0.8)

    # 2) 실주행 궤적 (전진/후진/대기 구분 표시)
    # 전진, 후진, 대기 구간 분리
    df_fwd  = track[track["mode"] == "fwd"]
    df_back = track[track["mode"] == "back"]
    df_wait = track[track["mode"] == "wait"]

    if not df_fwd.empty:
        plt.plot(df_fwd["x"], df_fwd["y"], linewidth=2, label="Trajectory (Forward)")
    if not df_back.empty:
        plt.plot(df_back["x"], df_back["y"], linewidth=2, linestyle="--", label="Trajectory (Back)")
    if not df_wait.empty:
        plt.scatter(df_wait["x"], df_wait["y"], s=40, marker="x", label="Wait points")

    # 3) 방향 화살표 (Yaw) - 너무 많으면 간격 샘플링
    N = len(track)
    step = max(N // 50, 1)  # 최대 50개 화살표
    xs = track["x"].values[::step]
    ys = track["y"].values[::step]
    yaws = track["yaw"].values[::step]  # rad
    u = np.cos(yaws)
    v = np.sin(yaws)
    plt.quiver(xs, ys, u, v, angles='xy', scale_units='xy', scale=5, width=0.003, alpha=0.5, label="Heading (sampled)")

    # 4) 범례 및 저장
    plt.legend(loc="best")
    out_path = os.path.abspath(args.out)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"[PLOT] 저장 완료: {out_path}")
    plt.show()

if __name__ == "__main__":
    main()
