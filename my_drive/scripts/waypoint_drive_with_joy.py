#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Joy
import sys
import os

# 전역 변수 설정
waypoints = []
gps_pose = Pose2D()
is_data_available = False
joy_data = Joy()
last_button_state = 0  # 이전에 눌린 버튼 상태를 저장

def utm_callback(data):
    """
    /gps/utm_pos1 토픽으로부터 데이터를 수신하는 콜백 함수.
    """
    global gps_pose
    global is_data_available
    gps_pose = data
    is_data_available = True

def joy_callback(data):
    """
    /joy 토픽으로부터 조이스틱 데이터를 수신하는 콜백 함수.
    """
    global joy_data
    joy_data = data

def save_waypoints_to_csv():
    """
    웨이포인트 리스트를 CSV 파일로 저장하는 함수.
    """
    if waypoints:
        filename = os.path.join(os.path.expanduser('~'), 'waypoints.csv')
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y']) # 헤더 작성
            writer.writerows(waypoints)
        rospy.loginfo(f"웨이포인트가 {filename} 파일에 성공적으로 저장되었습니다.")
    else:
        rospy.loginfo("저장할 웨이포인트가 없습니다.")

def waypoint_recorder():
    """
    ROS 노드 메인 함수.
    """
    global waypoints
    global last_button_state
    
    rospy.init_node('waypoint_recorder', anonymous=True)
    rospy.Subscriber("/gps/utm_pos1", Pose2D, utm_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)

    rospy.loginfo("웨이포인트 레코더가 시작되었습니다. 조이스틱의 버튼 LB(A버튼)을 눌러 좌표를 저장하세요. Ctrl+C를 눌러 종료하고 저장합니다.")

    rate = rospy.Rate(10)  # 10Hz로 루프 실행
    
    try:
        while not rospy.is_shutdown():
            if len(joy_data.buttons) > 0:
                current_button_state = joy_data.buttons[6]  # LB버튼임(다른 버튼들은 주행하면서 누르기 쉽지 않을 것 같음)                
                # 버튼을 누른 순간(이전 상태는 0, 현재 상태는 1)에만 웨이포인트 저장
                if current_button_state == 1 and last_button_state == 0:
                    if is_data_available:
                        current_waypoint = [gps_pose.x, gps_pose.y]
                        waypoints.append(current_waypoint)
                        index = len(waypoints) - 1
                        rospy.loginfo(f"웨이포인트 #{index} 추가: x={gps_pose.x}, y={gps_pose.y}")
                    else:
                        rospy.logwarn("GPS 데이터가 아직 수신되지 않았습니다. 잠시 기다려주세요.")
                
                last_button_state = current_button_state

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        save_waypoints_to_csv()

if __name__ == '__main__':
    try:
        waypoint_recorder()
    except rospy.ROSInterruptException:
        pass
