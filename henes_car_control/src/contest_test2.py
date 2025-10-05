#! /usr/bin/env python3

import numpy as np
# import torch
import pandas as pd
import cv2
import os
import datetime
import os.path as osp
import glob
import argparse
import math
import random
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
current_time = datetime.datetime.now()
formatted_time = current_time.strftime("%m_%d_%H_%M")

flag = 3

def save_models(name, test_num,num,img):
    if not os.path.exists('/home/ubuntu/contest/' + name + '/' + str(test_num) +"/"):
        os.makedirs('/home/ubuntu/contest/' + name + '/' + str(test_num) +"/")
    cv2.imwrite('/home/ubuntu/contest/' + name + '/' + str(test_num) +"/"+str(num) + '.png', img)


def cal_ang(pt1,pt2):
    sub_x = pt1[0]-pt2[0]# up-down
    sub_y = abs(pt1[1]-pt2[1])
    cal_ang=math.atan(sub_x/sub_y)
    cal_ang=(cal_ang*180)/math.pi
    return cal_ang
######################ROI click###############################
pts_cnt = 0
pts=[[],[],[],[],[],[],[],[]]
def on_mouse(event, x, y, flags, param):
    
    global pts_cnt,pts

    if event == cv2.EVENT_LBUTTONDOWN: # 왼쪽 클릭시 실행
        pts[pts_cnt] = [x, y]
        print(pts)
        pts_cnt += 1

def onChange(x):
    pass

def edge_tracking(): # bird eye view check point
    blur_w=5
    cap = cv2.VideoCapture('/dev/v4l/by-path/pci-0000:00:14.0-usb-0:5.4.1.1:1.0-video-index0')
    cv2.namedWindow("edge tracking")
    cv2.createTrackbar("L_thr","edge tracking",50,1400,onChange)
    cv2.createTrackbar("H_thr","edge tracking",50,1400,onChange)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if frame is None:
            print("no frame")
            break  
        #cv2.imshow("frame",frame)
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            pts1=np.float32([np.S_lay_L,np.S_lay_R, np.Fi_lay_R, np.Fi_lay_L])# total ROI
            pts2=np.float32([[0,0],[300,0],[300,300],[0,300]])
            
            T_ROI=cv2.getPerspectiveTransform(pts1,pts2)# total ROI
            T_dst=cv2.warpPerspective(frame,T_ROI,(300,300))#
            
            #cv2.imshow("frame",frame)
            #cv2.imshow('wb',T_dst)
            low=cv2.getTrackbarPos("L_thr","edge tracking")
            high=cv2.getTrackbarPos("H_thr","edge tracking")
            frame=cv2.GaussianBlur(frame,(blur_w,blur_w),0)
            frame=cv2.Canny(frame,low,high)
            dst=cv2.GaussianBlur(T_dst,(blur_w,blur_w),0)
            dst=cv2.Canny(dst,low,high)
            kernel = np.ones((15, 15), np.uint8)# change kernel size
        
            T_morph = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel)
            frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
            cv2.imshow('dst',T_morph)
            cv2.imshow('fra',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def laneDetCallback(msg):
    global flag
    flag = msg.data
    # print("lanedet : ",flag)

########topic 통신###################
rospy.init_node('henes_lane', anonymous=True)
drive_pub = rospy.Publisher('henes_lane', Twist, queue_size=1)
driving_flag_sub = rospy.Subscriber(name="/waypoint_flag", data_class=Int32, callback=laneDetCallback)
############parameter###############
# short roi
np.Fi_lay_R=[630, 472]
np.S_lay_R=[500, 334]

np.Fi_lay_L=[9, 472]
np.S_lay_L=[139, 334]

np.Fi_lay_M=[(630+9)/2,472]
np.S_lay_M=[(500+139)/2,334]

# edge_tracking()

total_list=[]#save to .csv

frame_num = 0 #몇 frame 마디 edge 추출할지
test_num=formatted_time# episode

num=0# save frame
speed_num=0#
Right_mean_ang=0
Left_mean_ang=0

blur_w=5
BI_threshold1=100
BI_threshold2=230
side_threshold1=100
side_threshold2=230
max_steeringAng=30
weight=1
speed=55

cap = cv2.VideoCapture('/dev/v4l/by-path/pci-0000:00:14.0-usb-0:5.4.1.1:1.0-video-index0')
# cap= cv2.VideoCapture("/home/ubuntu/contest/video/output05_02_18_12.avi")
########video save###################
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS) 
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('/home/ubuntu/contest/video/output'+str(test_num)+'.avi', fourcc, fps, (int(width), int(height)))

while cap.isOpened():
    if flag==3:
        _, frame = cap.read()
        if frame is None:
            print("no frame")
            break  
        out.write(frame)    
        # cv2.imshow('wb',frame)
        if frame_num%5 == 0:
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            wide_L = gray_img[334:472,0:139].copy()
            wide_R = gray_img[334:472,500:639].copy()
            half_roi = gray_img[320:479,:].copy()
            
            save_models('wide_L', test_num,num,wide_L)
            save_models('wide_R', test_num,num,wide_R)
        ####################4########################################

            #cv2.setMouseCallback('wb', on_mouse)
            
        #################bird eye view###############################
            pts1=np.float32([np.S_lay_L,np.S_lay_R, np.Fi_lay_R, np.Fi_lay_L])# total ROI
            pts2=np.float32([[0,0],[300,0],[300,300],[0,300]])
            pts3=np.float32([np.S_lay_L,np.S_lay_M, np.Fi_lay_M, np.Fi_lay_L])# left ROI
            pts4=np.float32([np.S_lay_M,np.S_lay_R, np.Fi_lay_R, np.Fi_lay_M])# right ROI
            pts5=np.float32([[0,0],[150,0],[150,300],[0,300]])
            
            T_ROI=cv2.getPerspectiveTransform(pts1,pts2)# total ROI
            L_ROI=cv2.getPerspectiveTransform(pts3,pts5)# left ROI
            R_ROI=cv2.getPerspectiveTransform(pts4,pts5)# right ROI
            
            T_dst=cv2.warpPerspective(gray_img,T_ROI,(300,300))#
            L_dst=cv2.warpPerspective(gray_img,L_ROI,(150,300))#
            R_dst=cv2.warpPerspective(gray_img,R_ROI,(150,300))#
        #################edge filter#################################
            T_blur=cv2.GaussianBlur(T_dst,(blur_w,blur_w),0)
            L_blur=cv2.GaussianBlur(L_dst,(blur_w,blur_w),0)
            R_blur=cv2.GaussianBlur(R_dst,(blur_w,blur_w),0)
            Half_blur=cv2.GaussianBlur(half_roi,(blur_w,blur_w),0)
            out_l_blur=cv2.GaussianBlur(wide_L,(blur_w,blur_w),0)
            out_r_blur=cv2.GaussianBlur(wide_R,(blur_w,blur_w),0)
            
            T_edge=cv2.Canny(T_blur, BI_threshold1, BI_threshold2)
            L_edge=cv2.Canny(L_blur, BI_threshold1, BI_threshold2)
            R_edge=cv2.Canny(R_blur, BI_threshold1, BI_threshold2)
            
            Half_edge=cv2.Canny(Half_blur, side_threshold1, side_threshold2)
            out_l_edge=cv2.Canny(out_l_blur, side_threshold1, side_threshold2)
            out_r_edge=cv2.Canny(wide_R, side_threshold1, side_threshold2)
            
            save_models('Half_edge', test_num,num,Half_edge)
            
        #################image opening closing#######################
            kernel = np.ones((15, 15), np.uint8)# change kernel size
            
            T_morph = cv2.morphologyEx(T_edge, cv2.MORPH_CLOSE, kernel)# NO morph open
            L_morph = cv2.morphologyEx(L_edge, cv2.MORPH_CLOSE, kernel)# NO morph open
            R_morph = cv2.morphologyEx(R_edge, cv2.MORPH_CLOSE, kernel)# NO morph open
            out_L_morph = cv2.morphologyEx(out_l_edge, cv2.MORPH_CLOSE, kernel)# NO morph open
            out_R_morph = cv2.morphologyEx(out_r_edge, cv2.MORPH_CLOSE, kernel)# NO morph open
            #cv2.imshow("l",L_morph)
            #cv2.imshow("r",R_morph)
            cv2.imshow("t",T_morph)
            save_models('Total morph', test_num,num,T_morph)
            save_models('Total wb', test_num,num,T_dst)
            save_models('Total edge', test_num,num,T_edge)
        ##################bird view blob detection##################################
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
            # Detect blobs in the grayscale image
            T_keypoints = detector.detect(T_morph)
            L_keypoints = detector.detect(L_morph)
            R_keypoints = detector.detect(R_morph)
            
            T_keypoint_coords = []
            L_keypoint_coords = []
            R_keypoint_coords = []
            
            for keypoint in T_keypoints:
                x = int(keypoint.pt[0])
                y = int(keypoint.pt[1])
                T_keypoint_coords.append((x, y))
            
            for keypoint in L_keypoints:
                x = int(keypoint.pt[0])
                y = int(keypoint.pt[1])
                L_keypoint_coords.append((x, y))

            for keypoint in R_keypoints:
                x = int(keypoint.pt[0])
                y = int(keypoint.pt[1])
                R_keypoint_coords.append((x, y))
    ##################side blob detection##################################
            params_side = cv2.SimpleBlobDetector_Params()
            params_side.filterByArea = True
            params_side.filterByConvexity = False
            params_side.filterByCircularity = False
            params_side.filterByInertia = False
            params_side.filterByColor = False
            params_side.minThreshold = 253
            params_side.maxThreshold = 255
            params_side.thresholdStep = 1
            params_side.minArea = 1
            params_side.maxArea = 500
            detector_side = cv2.SimpleBlobDetector_create(params_side)
            # Detect blobs in the grayscale image
            out_l_keypoints = detector_side.detect(out_L_morph)
            out_r_keypoints = detector_side.detect(out_R_morph)
            
            out_l_keypoint_coords = []
            out_r_keypoint_coords = []
            
            for keypoint in out_l_keypoints:
                x = int(keypoint.pt[0])
                y = int(keypoint.pt[1])
                out_l_keypoint_coords.append((x, y))

            for keypoint in out_r_keypoints:
                x = int(keypoint.pt[0])
                y = int(keypoint.pt[1])
                out_r_keypoint_coords.append((x, y))
                
            #for x, y in out_l_keypoint_coords:
            #    cv2.circle(out_L_morph, (x, y), 3, 0, -1)
            #for x, y in out_r_keypoint_coords:
            #    cv2.circle(out_R_morph, (x, y), 3, 0, -1)
            ########################countours##########################

            T_contours, _ = cv2.findContours(T_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            L_contours, _ = cv2.findContours(L_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            R_contours, _ = cv2.findContours(R_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            out_L_contours, _ = cv2.findContours(out_L_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            out_R_contours, _ = cv2.findContours(out_R_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            
            Data_list = np.zeros([10])
            
            Left_diff_ang=[]
            Left_real_ang=[]
            Right_diff_ang=[]
            Right_real_ang=[]
            
            L_line=[]
            R_line=[]
            T_line=[]
            out_L_line=[]
            out_R_line=[]
            L_middle_line=[]
            R_middle_line=[]
            T_middle_line=[]
            out_L_middle_line=[]
            out_R_middle_line=[]
            L_distance=[]
            R_distance=[]
            T_distance=[]
            out_L_distance=[]
            out_R_distance=[]
            camera_M=[]
    #####################out side_left lane#########################
            for j in range(len(out_L_contours)):
                cts=[]
                for i in range(len(out_L_contours[j])):
                    cts.append(out_L_contours[j][i][0][1])
                cts_max=cts.index(max(cts))
                cts_min=cts.index(min(cts))
                out_L_line.append([out_L_contours[j][cts_max][0], out_L_contours[j][cts_min][0]])
                out_L_middle_line.append((out_L_line[j][0]+out_L_line[j][1])/2)
                
            for i in range(len(out_l_keypoint_coords)):
                L_out_wb_distance=[]
                for j in range(len(out_L_middle_line)):
                    L_out_wb_distance.append(math.sqrt((out_l_keypoint_coords[i][0]-out_L_middle_line[j][0])**2+(out_l_keypoint_coords[i][1]-out_L_middle_line[j][1])**2))
                out_L_distance.append(L_out_wb_distance.index(min(L_out_wb_distance)))
            if len(out_L_line)!=0:     
                for i in sorted(set(out_L_distance), reverse = True):
                    del out_L_line[i]
            for i in range(len(out_L_line)):    
                cv2.line(out_l_blur, out_L_line[i][0], out_L_line[i][1], (255,255,0), thickness=3)
    #####################out side_right lane#########################
            
            for j in range(len(out_R_contours)):
                cts=[]
                for i in range(len(out_R_contours[j])):
                    cts.append(out_R_contours[j][i][0][1])
                cts_max=cts.index(max(cts))
                cts_min=cts.index(min(cts))
                out_R_line.append([out_R_contours[j][cts_max][0], out_R_contours[j][cts_min][0]])
                out_R_middle_line.append((out_R_line[j][0]+out_R_line[j][1])/2)
            for i in range(len(out_r_keypoint_coords)):
                R_out_wb_distance=[]
                for j in range(len(out_R_middle_line)):
                    R_out_wb_distance.append(math.sqrt((out_r_keypoint_coords[i][0]-out_R_middle_line[j][0])**2+(out_r_keypoint_coords[i][1]-out_R_middle_line[j][1])**2))
                out_R_distance.append(R_out_wb_distance.index(min(R_out_wb_distance)))
            if len(out_R_line)!=0:     
                for i in sorted(set(out_R_distance), reverse = True):
                    del out_R_line[i]
            for i in range(len(out_R_line)):    
                cv2.line(out_r_blur, out_R_line[i][0], out_R_line[i][1], (255,255,0), thickness=3)
            save_models('out_l_edge', test_num,num,out_l_blur)
            save_models('out_r_edge', test_num,num,out_r_blur)
        ##########################left lane#########################
            for j in range(len(L_contours)):
                cts=[]
                for i in range(len(L_contours[j])):
                    cts.append(L_contours[j][i][0][1])
                cts_max=cts.index(max(cts))
                cts_min=cts.index(min(cts))
                L_line.append([L_contours[j][cts_max][0], L_contours[j][cts_min][0]])
                L_middle_line.append((L_line[j][0]+L_line[j][1])/2)
                
            for i in range(len(L_keypoint_coords)):
                L_wb_distance=[]
                for j in range(len(L_middle_line)):
                    L_wb_distance.append(math.sqrt((L_keypoint_coords[i][0]-L_middle_line[j][0])**2+(L_keypoint_coords[i][1]-L_middle_line[j][1])**2))
                L_distance.append(L_wb_distance.index(min(L_wb_distance)))
            
            if len(L_line)!=0:     
                for i in sorted(set(L_distance), reverse = True):
                    del L_line[i]
            
            for i in range(len(L_line)):    
                Left_diff_ang.append(cal_ang(L_line[i][1],L_line[i][0]))
            
            for k in range(len(L_line)):
                if abs(Left_diff_ang[k]) < (40):
                    Left_real_ang.append(Left_diff_ang[k])
            if len(Left_real_ang)==0:
                # print("len(Left_real_ang)==0")
                Left_mean_ang=0
            else:
                Left_mean_ang=sum(Left_real_ang)/len(Left_real_ang)
        
            for i in range(len(L_line)):    
                cv2.line(L_dst, L_line[i][0], L_line[i][1], (255,255,0), thickness=30)
                camera_M.append(L_line[i][0])
                camera_M.append(L_line[i][1])
        #####################right lane#################################
            for j in range(len(R_contours)):
                cts=[]
                for i in range(len(R_contours[j])):
                    cts.append(R_contours[j][i][0][1])
                cts_max=cts.index(max(cts))
                cts_min=cts.index(min(cts))
                R_line.append([R_contours[j][cts_max][0], R_contours[j][cts_min][0]])
                R_middle_line.append((R_line[j][0]+R_line[j][1])/2)

            for i in range(len(R_keypoint_coords)):
                R_wb_distance=[]
                for j in range(len(R_middle_line)):
                    R_wb_distance.append(math.sqrt((R_keypoint_coords[i][0]-R_middle_line[j][0])**2+(R_keypoint_coords[i][1]-R_middle_line[j][1])**2))
                R_distance.append(R_wb_distance.index(min(R_wb_distance)))
            
            if len(R_line)!=0:    
                for i in sorted(set(R_distance), reverse = True):
                    del R_line[i]
                
            for i in range(len(R_line)):    
                Right_diff_ang.append(cal_ang(R_line[i][1],R_line[i][0]))
            
            for k in range(len(R_line)):
                if abs(Right_diff_ang[k]) < (40):
                    Right_real_ang.append(Right_diff_ang[k])
            
            if len(Right_real_ang)==0:
                # print("len(Right_real_ang)==0")
                Right_mean_ang=0    
            else:
                Right_mean_ang=sum(Right_real_ang)/len(Right_real_ang) 
            
            for i in range(len(R_line)):    
                cv2.line(R_dst, R_line[i][0], R_line[i][1], (255,255,0), thickness=30)
                camera_M.append(R_line[i][0])
                camera_M.append(R_line[i][1])
            ##################TOTAL LANE##################
            for j in range(len(T_contours)):
                cts=[]
                for i in range(len(T_contours[j])):
                    cts.append(T_contours[j][i][0][1])
                cts_max=cts.index(max(cts))
                cts_min=cts.index(min(cts))
                T_line.append([T_contours[j][cts_max][0], T_contours[j][cts_min][0]])
                T_middle_line.append((T_line[j][0]+T_line[j][1])/2)
                
            for i in range(len(T_keypoint_coords)):
                T_wb_distance=[]
                for j in range(len(T_middle_line)):
                    T_wb_distance.append(math.sqrt((T_keypoint_coords[i][0]-T_middle_line[j][0])**2+(T_keypoint_coords[i][1]-T_middle_line[j][1])**2))
                T_distance.append(T_wb_distance.index(min(T_wb_distance)))
            
            if len(T_line)!=0:
                for i in sorted(set(T_distance), reverse = True):
                    del T_line[i]
            ##################drive control###############
            drive_data = Twist()
            if len(T_line) == 1 and len(out_L_line) != 0 and T_line[0][0][0] > 150 :
                if Right_mean_ang == 0:
                    mean_ang = Left_mean_ang
                elif Left_mean_ang == 0:
                    mean_ang = Right_mean_ang        
                
                drive_data.linear.x = speed
                drive_data.angular.z = -50
                # drive_data.linear.y = 1
                # if abs(mean_ang) > 5:
                #     # drive_data.angular.z = mean_ang*weight
                #     drive_data.angular.z = -30
                # else:
                #     drive_data.angular.z = -30
                    
            elif len(T_line) == 1 and len(out_R_line) != 0 and T_line[0][0][0] < 150:
                if Right_mean_ang == 0:
                    mean_ang = Left_mean_ang
                elif Left_mean_ang == 0:
                    mean_ang = Right_mean_ang
                
                drive_data.linear.x = speed
                drive_data.angular.z = 50
                # drive_data.linear.y = 1
                # if abs(mean_ang) > 5:
                #     # drive_data.angular.z = mean_ang*weight
                #     drive_data.angular.z = 30
                # else:
                #     drive_data.angular.z = 30
            elif len(T_line) == 0:
                drive_data.linear.x = speed
                # drive_data.angular.z = 0
                # drive_data.linear.y = 0
                mean_ang =0
            else:
                middle_lane=np.mean(camera_M)
                print(middle_lane)

                if Right_mean_ang == 0:
                    mean_ang = Left_mean_ang
                elif Left_mean_ang == 0:
                    mean_ang = Right_mean_ang
                else:
                    mean_ang=(Right_mean_ang+Left_mean_ang)/2
                
                drive_data.angular.z = mean_ang - (middle_lane-150)*0.1
                drive_data.linear.x = speed
                # drive_data.linear.y = 1
                # drive_data.angular.z = mean_ang*weight
            #################확 꺾일 때 예외처리#######################
                #if mean_ang >= 0:
                #    if mean_ang - past_mean_ang >  15:
                #        mean_ang = mean_ang - (mean_ang-past_mean_ang)/2
                #else:
                #    if past_mean_ang - mean_ang >  15:
                #        mean_ang = mean_ang + (past_mean_ang - mean_ang)/2
                ###############차량 속도 제어##############################
                
                #if speed_num < 3:
                #drive_data.linear.x = 40
                #    speed_num+=1
                #else:
                #    drive_data.linear.x = 25
                
                #########topic 통신###################
                

                #past_mean_ang = mean_ang
            # cv2.imshow("L_dst",L_dst)
            # cv2.imshow("R_dst",R_dst)
            save_models('draw right', test_num,num,R_dst)
            save_models('draw left', test_num,num,L_dst)
            Data_list[0]=Left_mean_ang
            Data_list[1]=Right_mean_ang
            Data_list[2]=len(Left_real_ang)
            Data_list[3]=len(Right_real_ang)
            Data_list[4]=mean_ang
            Data_list[5]=drive_data.linear.x
            Data_list[6]=drive_data.angular.z
            Data_list[7]=len(T_line)
            Data_list[8]=len(out_L_line)
            Data_list[9]=len(out_R_line)
            #Data_list[10]=T_line[0][0][0]
            total_list.append(Data_list)
            
            #rospy.loginfo(drive_data)
            drive_pub.publish(drive_data)
            num+=1
    #############################################################
        #cv2.imshow('Left_morph ',L_morph)
        #cv2.imshow('Right_morph',R_morph)
        frame_num+=1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

df = pd.DataFrame(total_list, columns=['left_ang','right_ang','len_left','len_right','mean_ang','linear_x','ang_z','len(T_line)','len(out_L_line)','len(out_R_line)'])
df.to_csv('/home/ubuntu/contest/dataset/'+str(test_num)+'.csv')
