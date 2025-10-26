import numpy as np
# import pandas as pd
import cv2
import os
import datetime
import math
import rospy
from std_msgs.msg import Float64,Int32
from geometry_msgs.msg import Twist

current_time = datetime.datetime.now()
formatted_time = current_time.strftime("%m_%d_%H_%M")
num = 0
frame_num = 0

def save_models(name, test_num,num,img):
    if not os.path.exists('/home/ubuntu/contest/' + name + '/' + str(test_num) +"/"):
        os.makedirs('/home/ubuntu/contest/' + name + '/' + str(test_num) +"/")
    cv2.imwrite('/home/ubuntu/contest/' + name + '/' + str(test_num) +"/"+str(num) + '.png', img)

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_pixel = hsv[y, x]
        print(f"H:{hsv_pixel[0]}, S:{hsv_pixel[1]}, V:{hsv_pixel[2]}")
        # save_models('img', formatted_time, num, frame)


cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("can't open cam.")
    exit()

cv2.namedWindow('wb')
cv2.namedWindow('od')
cv2.namedWindow('hsv')
cv2.setMouseCallback('hsv', on_mouse)

np.Fi_lay_R=[630, 472]
np.S_lay_R=[500, 334]

np.Fi_lay_L=[9, 472]
np.S_lay_L=[139, 334]

np.Fi_lay_M=[(630+9)/2,472]
np.S_lay_M=[(500+139)/2,334]

points = np.array([[139, 334], [500, 334], [630, 472], [9, 472]], np.int32)
points = points.reshape((-1, 1, 2))

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('/home/ubuntu/contest/video/output' + str(formatted_time) + '.avi', fourcc, fps,
                      (int(width), int(height)))

while True:
    ret, frame = cap.read()

    if not ret:
        print("no video frame")
        break
    out.write(frame)
    if frame_num % 5 == 0:
        cv2.polylines(frame, [points], isClosed = True, color = (0, 255, 0), thickness = 2)
        cv2.imshow('wb', frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow('hsv', hsv)

        lower_orange = np.array([15, 55, 220])    #[15, 30, 140]
        upper_orange = np.array([35, 100, 255])     #[40, 150, 255]

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        orange_detected = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('od', orange_detected)

        num += 1


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()