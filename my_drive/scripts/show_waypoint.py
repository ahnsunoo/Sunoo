#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# 웨이포인트 CSV 불러오기
waypoints = pd.read_csv("/home/icas/p_parking_A.csv")

# x, y 좌표 추출
x = waypoints["x"]
y = waypoints["y"]

# 시각화
plt.figure(figsize=(8, 6))
plt.plot(x, y, marker="o", linestyle="-", label="Waypoints path")
plt.scatter(x.iloc[0], y.iloc[0], c="green", s=100, marker="o", label="Start")  # 시작점
plt.scatter(x.iloc[-1], y.iloc[-1], c="red", s=100, marker="s", label="Goal")   # 목표점

# 웨이포인트 번호 표시
for i, (px, py) in enumerate(zip(x, y)):
    plt.text(px, py+0.2, str(i), fontsize=8, ha="center")

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Waypoints Visualization")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
