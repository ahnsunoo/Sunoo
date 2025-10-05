import numpy as np
import cv2
import math
import os
import datetime
# import rospy
# from geometry_msgs.msg import Twist

current_time = datetime.datetime.now()
formatted_time = current_time.strftime("%m_%d_%H_%M")

flag = 3

def save_models(name, test_num, num, img):
    if not os.path.exists('/home/ubuntu/contest/' + name + '/' + str(test_num) + "/"):
        os.makedirs('/home/ubuntu/contest/' + name + '/' + str(test_num) + "/")
    cv2.imwrite('/home/ubuntu/contest/' + name + '/' + str(test_num) + "/" + str(num) + '.png', img)

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_pixel = hsv[y, x]
        print(f"H:{hsv_pixel[0]}, S:{hsv_pixel[1]}, V:{hsv_pixel[2]}")

def cal_ang(pt1, pt2):
    sub_x = pt2[0] - pt1[0]  # up-down
    sub_y = pt2[1] - pt1[1]
    if (sub_x == 0) or (abs(sub_x) < 20):
        ang = 0
        return ang
    ang = math.atan(sub_y / sub_x)
    ang = (-1) * (ang * 180) / math.pi
    return ang

# rospy.loginfo("initializing")
# rospy.init_node('henes_lane', anonymous=True)
# drive_pub = rospy.Publisher('/lane_cmd_vel', Twist, queue_size=1)
# drive_data = Twist()

cv2.namedWindow('hsv')
cv2.setMouseCallback('hsv', on_mouse)

frame_num = 0  # 몇 frame 마디 edge 추출할지
test_num = formatted_time  # episode

num = 0  # save frame

lower_orange = np.array([15, 40, 200])    # [15, 40, 200], [15, 70, 220], [15, 30, 220]
upper_orange = np.array([40, 80, 255])     # [40, 80, 255], [40, 130, 255], [40, 90, 255]
blur_w = 29
BI_threshold1 = 150
BI_threshold2 = 180
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (51, 71))
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


speed = 50
weight = 2

video = cv2.VideoCapture("./lane_video1.avi")
frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))


for i in range(frame_count):
    if flag == 3:
        _, frame = video.read()
        if frame is None:
            print("no frame")
            break
        # out.write(frame)


        if frame_num % 5 == 0:

            line = []  # 각도 포인트 및 각도 저장
            ang = []
            mean_ang = 0

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)            # 주황색 검출하여 255, 나머지 0
            img_org = cv2.inRange(hsv, lower_orange, upper_orange)
            cv2.imshow('hsv', hsv)
            cv2.imshow('img_org', img_org)

            keypoints = detector.detect(img_org)

            # Blob 제거
            for keypoint in keypoints:
                center = tuple(map(int, keypoint.pt))
                radius = 4
                cv2.circle(img_org, center, radius, 0, -1)

            cv2.imshow('blob_img', img_org)

            img_blur = cv2.GaussianBlur(img_org, (blur_w, blur_w), 0)
            img_edge = cv2.Canny(img_blur, BI_threshold1, BI_threshold2)

            img_morph = cv2.morphologyEx(img_edge, cv2.MORPH_CLOSE, kernel)

            img_contours, _ = cv2.findContours(img_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

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


            print(line)
            print(mean_ang * weight)

            # drive_data.linear.x = speed
            # drive_data.angular.z = mean_ang * weight
            # drive_pub.publish(drive_data)

            # cv2.imshow('frame', frame)
            cv2.drawContours(frame, img_contours, -1, (255, 0, 0), thickness=3)
            cv2.imshow("test_contours", frame)
            key = cv2.waitKey(0) & 0xFF

            # n 키를 누를 때마다 다음 프레임으로 이동
            if key == ord('n'):
                continue

            # q 키를 누르면 종료
            elif key == ord('q'):
                break
            if (len(line)):
                bf_y = line[0][0][1]

            num += 1

        frame_num += 1
#
# cv2.waitKey(0)
cv2.destroyAllWindows()
video.release()