import numpy as np
import cv2
import math
import os
import datetime
# import rospy
# from geometry_msgs.msg import Twist

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_pixel = hsv[y, x]
        print(f"H:{hsv_pixel[0]}, S:{hsv_pixel[1]}, V:{hsv_pixel[2]}")

def cal_ang(pt1, pt2):
    sub_x = pt2[0] - pt1[0]  # right - left
    sub_y = pt2[1] - pt1[1]
    ang = math.atan(sub_y / sub_x)
    ang = (-1) * (ang * 180) / math.pi
    return ang

# rospy.init_node('henes_lane', anonymous=True)
# drive_pub = rospy.Publisher('henes_lane', Twist, queue_size=1)
# drive_data = Twist()

speed = 50
weight = 2

img = cv2.imread("./45.png")
cv2.imshow("test", img)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# cv2.imshow("hsv", hsv)
lower_orange = np.array([15, 70, 210])    #[15, 30, 140]
upper_orange = np.array([40, 130, 255])     #[40, 150, 255]
img_org = cv2.inRange(hsv, lower_orange, upper_orange)
cv2.imshow("test_orange", img_org)

cv2.namedWindow("HSV Viewer")
cv2.setMouseCallback("HSV Viewer", on_mouse)

while True:
    cv2.imshow("HSV Viewer", hsv)
    key = cv2.waitKey(1) & 0xFF

    # 'q' 키를 누르면 종료
    if key == ord('q'):
        break

# img_rgb = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
# cv2.imshow("test_rgb", img_rgb)

# img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
# cv2.imshow("test_gray", img_gray)

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
params.maxArea = 500
detector = cv2.SimpleBlobDetector_create(params)

keypoints = detector.detect(img_org)

# Blob 제거
for keypoint in keypoints:
    center = tuple(map(int, keypoint.pt))
    radius = 2
    cv2.circle(img_org, center, radius, 0, -1)

cv2.imshow('blob_img', img_org)

blur_w = 25     # 17
img_blur = cv2.GaussianBlur(img_org, (blur_w, blur_w), 0)
cv2.imshow("test_blur", img_blur)

BI_threshold1 = 150     #150
BI_threshold2 = 180
img_edge = cv2.Canny(img_blur, BI_threshold1, BI_threshold2)
cv2.imshow("test_edge", img_edge)

# kernel = np.ones((37, 37), np.uint8)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (51, 71))
img_morph = cv2.morphologyEx(img_edge, cv2.MORPH_CLOSE, kernel)
cv2.imshow("test_morph", img_morph)


img_contours, _ = cv2.findContours(img_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

cv2.drawContours(img, img_contours, -1, (255, 0, 0), thickness=3)
cv2.imshow("test_contours", img)

line = []
ang = []
mean_ang = []

for i in range(len(img_contours)):
    cts = []
    for j in range(len(img_contours[i])):
        cts.append(img_contours[i][j][0][0])
    cts_min = cts.index(min(cts))
    cts_max = cts.index(max(cts))
    line.append([img_contours[i][cts_min][0], img_contours[i][cts_max][0]])

for i in range(len(line)):
    ang.append(cal_ang(line[i][0], line[i][1]))

if (len(ang)):
    mean_ang = sum(ang) / len(ang)
else:
    mean_ang = 0

print(line)
print(mean_ang)
# drive_data.linear.x = speed
# drive_data.angular.z = ang * weight
# drive_pub.publish(drive_data)


cv2.waitKey(0)
cv2.destroyAllWindows()
