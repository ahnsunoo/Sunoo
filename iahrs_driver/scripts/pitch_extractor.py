#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation as R

def imu_callback(msg):
    # 쿼터니언 -> 오일러 변환 (scipy 사용)
    q = msg.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    r = R.from_quat(quaternion)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True) # roll, pitch, yaw 추출

    # 추출한 pitch 값을 새로운 토픽으로 발행
    pitch_pub.publish(Float32(data=pitch))

def main():
    # 'imu_pitch_extractor' 라는 이름으로 노드 초기화
    rospy.init_node('imu_pitch_extractor')

    # 원본 IMU 데이터 토픽 구독
    rospy.Subscriber("/imu/data", Imu, imu_callback)

    # Publisher를 전역 변수로 선언
    global pitch_pub

    # '/imu/pitch' 라는 이름으로 Float32 타입의 토픽을 발행
    pitch_pub = rospy.Publisher("/imu/pitch", Float32, queue_size=10)
    rospy.loginfo("Pitch extractor started (Python 3 + scipy)")

    # 노드가 종료될 때까지 대기
    rospy.spin()

if __name__ == "__main__":
    main()