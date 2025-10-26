import numpy as np
import cv2
import math
import os
import datetime
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

current_time = datetime.datetime.now()
formatted_time = current_time.strftime("%m_%d_%H_%M")

lane_mode = 0
lane_stop = 0


def save_models(name, test_num, num, img):
    if not os.path.exists('/home/ubuntu/contest/' + name + '/' + str(test_num) + "/"):
        os.makedirs('/home/ubuntu/contest/' + name + '/' + str(test_num) + "/")
    cv2.imwrite('/home/ubuntu/contest/' + name + '/' + str(test_num) + "/" + str(num) + '.png', img)


def cal_ang(pt1, pt2):
    sub_x = pt2[0] - pt1[0]  # up-down
    sub_y = pt2[1] - pt1[1]
    if abs(sub_x) < 20:
        ang = 0
        return ang
    ang = math.atan(sub_y / sub_x)
    ang = (-1) * (ang * 180) / math.pi
    return ang
def GPS_callback(msg):
    global lane_mode
    if msg.position_covariance[0] > 1.0:
        lane_mode = 1
    else:
        lane_mode = 0

rospy.loginfo("initializing")
rospy.init_node('henes_lane', anonymous=True)
drive_pub = rospy.Publisher('/lane_cmd_vel', Twist, queue_size=1)
drive_data = Twist()
gps_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, callback=GPS_callback)

frame_num = 0
test_num = formatted_time  # episode

num = 0  # save frame

lower_orange = np.array([15, 55, 220])  # [15, 30, 140]
upper_orange = np.array([35, 100, 255])  # [40, 150, 255]
blur_w = 23
BI_threshold1 = 150
BI_threshold2 = 180
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 49))
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.filterByConvexity = False
params.filterByCircularity = False
params.filterByInertia = False
params.filterByColor = False
params.minThreshold = 253
params.maxThreshold = 255
params.thresholdStep = 1
params.minArea = 1
params.maxArea = 1000
detector = cv2.SimpleBlobDetector_create(params)


speed = 45
weight = 3
bf_y = 240

cap = cv2.VideoCapture(0)

########video save###################
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('/home/ubuntu/contest/video/output' + str(test_num) + '.avi', fourcc, fps,
                      (int(width), int(height)))

while cap.isOpened():
    # covariance 발산 -> lane_mode = 1
    # covariance 수렴 -> lane_mode = 0
    if lane_mode == 1:
        _, frame = cap.read()
        if frame is None:
            print("no frame")
            break
        out.write(frame)

        if frame_num % 5 == 0:

            line = []
            ang = []
            mean_ang = 0

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            img_orange = cv2.inRange(hsv, lower_orange, upper_orange)

            save_models('origin', test_num, num, frame)
            save_models('img_orange', test_num, num, img_orange)

            keypoints = detector.detect(img_orange)

            # Blob 제거
            for keypoint in keypoints:
                center = tuple(map(int, keypoint.pt))
                radius = 3
                cv2.circle(img_orange, center, radius, 0, -1)

            save_models('img_blob', test_num, num, img_orange)

            img_blur = cv2.GaussianBlur(img_orange, (blur_w, blur_w), 0)
            img_edge = cv2.Canny(img_blur, BI_threshold1, BI_threshold2)

            save_models('img_edge', test_num, num, img_edge)

            img_morph = cv2.morphologyEx(img_edge, cv2.MORPH_CLOSE, kernel)

            save_models('img_morph', test_num, num, img_morph)

            img_contours, _ = cv2.findContours(img_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            cv2.drawContours(frame, img_contours, -1, (255, 0, 0), thickness=3)

            save_models('img_contours', test_num, num, frame)

            if (len(img_contours)):
                for i in range(len(img_contours)):
                    cts = []
                    for j in range(len(img_contours[i])):
                        cts.append(img_contours[i][j][0][0])
                    cts_min = cts.index(min(cts))
                    cts_max = cts.index(max(cts))
                    line.append([img_contours[i][cts_min][0], img_contours[i][cts_max][0]])

                for i in range(len(line)):
                    ang.append(cal_ang(line[i][0], line[i][1]))

                mean_ang = sum(ang) / len(ang)

            else:
                if num:
                    if (bf_y <= 100):
                        mean_ang = 50
                    elif (bf_y >= 380):
                        mean_ang = -50



            drive_data.linear.x = speed
            drive_data.angular.z = mean_ang * weight
            drive_pub.publish(drive_data)
            # print(drive_data.angular.z)

            if (len(line)):
                bf_y = line[0][0][1]

            num += 1

        frame_num += 1

    else:
        lane_stop = 1

