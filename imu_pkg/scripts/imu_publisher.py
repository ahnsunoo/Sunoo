#!/usr/bin/env python3.8
# -*- coding:utf-8 -*-
import serial
import time
import rospy
from std_msgs.msg import Float32

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_SPEED = 115200
COMM_RECV_TIMEOUT = 30  # 밀리초 단위

def open_serial():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_SPEED,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0
        )
        print(f"시리얼 포트를 성공적으로 열었습니다: {SERIAL_PORT}")
        return ser
    except serial.SerialException as e:
        print(f"오류: {SERIAL_PORT}을(를) 열 수 없습니다: {e}")
        return None

def send_recv(ser, command, data_length):
    ser.reset_input_buffer()
    ser.write(command.encode('utf-8'))

    recv_buff = bytearray()
    time_start = time.time()

    while True:
        if ser.in_waiting > 0:
            recv_buff.extend(ser.read(ser.in_waiting))
            if recv_buff[-1] == ord('\r') or recv_buff[-1] == ord('\n'):
                break

        time_elapsed = (time.time() - time_start) * 1000
        if time_elapsed >= COMM_RECV_TIMEOUT:
            break

        time.sleep(0.001)  # 1ms delay

    recv_str = recv_buff.decode('utf-8').strip()

    if len(recv_str) > 0 and recv_str[0] == '!':
        return -1

    if recv_str.startswith(command.strip()):
        data_str = recv_str[len(command.strip()):].lstrip('=')
        data_values = data_str.split(',')

        returned_data = []
        for val in data_values:
            if val.startswith("0x"):
                returned_data.append(int(val, 16))
            else:
                try:
                    returned_data.append(float(val))
                except ValueError:
                    break
        
        return returned_data[:data_length]

    return []

def main():
    rospy.init_node('imu_publisher')
    yaw_pub = rospy.Publisher('imu/yaw', Float32, queue_size=10)
    pitch_pub = rospy.Publisher('imu/pitch', Float32, queue_size=10)
    roll_pub = rospy.Publisher('imu/roll', Float32, queue_size=10)
    ser = open_serial()
    if ser:
        max_data = 10  # 최대 읽을 데이터 수
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            euler_data = send_recv(ser, "e\n", max_data)
            if len(euler_data) >= 3:
                roll, pitch, yaw = euler_data[0], euler_data[1], euler_data[2]
                ############################################################## offset input ###################################################3
                offset = -15 + 109
                new_yaw = (yaw + offset + 180) % 360 - 180
                yaw_pub.publish(new_yaw)
                pitch_pub.publish(pitch)
                roll_pub.publish(roll)
                rospy.loginfo(f"Euler Angles - Roll: {roll}")
                rospy.loginfo(f"Euler Angles - Pitch: {pitch}")
                rospy.loginfo(f"Euler Angles - Yaw: {yaw}")

            time.sleep(0.1)  # 100 ms delay

        ser.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
