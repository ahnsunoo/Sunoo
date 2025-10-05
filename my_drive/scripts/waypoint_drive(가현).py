# !/usr/bin/env python3

import math
import pandas as pd

#####ros통신을 위한 라이브러리 추가#######
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32



current_x = 0.0
current_y = 0.0

current_yaw = 0.0
current_pitch=0.0


velocity = Twist()
wheelbase=0.73 #차량 앞바퀴축 뒷바퀴축 사이의 거리
L = 2 # 목표와 현재위치와의 오차범위
kp_angular=1
back_point = [-1, -1]  # 후진을 해야하는 웨이포인트index
wait_point=[-1,-1]  #잠시 정지해야하는 웨이포인트 index
cnt=0       #wait상태일때 시간재려고 정의한 변수

have_gps = False          # [MOD] 첫 센서 수신 확인 플래그
have_imu = False          # [MOD]



def gps_callback(msg):
    """
    /utm 토픽에서 위치 데이터를 받아 전역 변수를 업데이트합니다.
    """
    global current_x, current_y, have_gps   # [MOD]
    current_x = msg.pose.position.x
    current_y = msg.pose.position.y
    have_gps = True                         # [MOD]
    rospy.loginfo("GPS Data received: x=%.2f, y=%.2f", current_x, current_y)


def imu_callback(msg):
    """
    /imu/yaw 토픽에서 yaw 각도 데이터를 받아 전역 변수를 업데이트합니다.
    """
    global current_yaw, have_imu            # [MOD]
    # IMU 각도가 도(degree) 단위라면 라디안으로 변환합니다.
    current_yaw = math.radians(msg.data)
    have_imu = True                         # [MOD]
    rospy.loginfo("IMU Data received: yaw=%.2f", current_yaw)


def getDistance(p1, p2):
    """
    Calculate distance
    :param p1: list, point1
    :param p2: list, point2
    :return: float, distance
    """
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)


class Trajectory:
    def __init__(self, traj_x, traj_y):
        """
        Define a trajectory class
        :param traj_x: list, list of x position
        :param traj_y: list, list of y position
        """
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0


    def getPoint(self, idx):
        return [self.traj_x[idx], self.traj_y[idx]]


    def getTargetPoint(self, pos):
        """
        Get the next look ahead point
        :param pos: list, 차량의 현재위치 (x,y)
        :return: list, target point
        """
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)

        if curr_dist < L and target_idx < len(self.traj_x) - 1:
            target_idx += 1

        self.last_idx = target_idx
        return self.getPoint(target_idx)


def main():

    global cnt
    rospy.init_node('gps_waypoint_controller')
    rate=rospy.Rate(10) #gps가 몇 hz로 받을 수 있는지 + 다른 센서들도 얼마의 주파수로 데이터 받는지에 따라 결정


###웨이포인트들 불러오기
    waypoint_file = pd.read_csv('/home/icas/latlon_to_utm.csv')

    traj_x = waypoint_file['x'].tolist()
    traj_y = waypoint_file['y'].tolist()
    if len(traj_x) == 0:
        rospy.logerr("Waypoint list is empty.")
        return


    # target course
    traj = Trajectory(traj_x, traj_y)
    goal = traj.getPoint(len(traj_x) - 1)


	

    velocity_pub = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/gps/utm_pos1', PoseStamped, gps_callback)
    rospy.Subscriber('/imu/yaw', Float32, imu_callback)

    rospy.sleep(0.2)

    # [MOD] 첫 GPS/IMU 수신 전까지 대기
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (not have_gps or not have_imu):
        if (rospy.Time.now() - t0).to_sec() > 2.0:
            rospy.logwarn_throttle(2.0, "[gps_waypoint_controller] waiting gps/imu...")
        rate.sleep()

    # [MOD] 시작 시 최근접 웨이포인트로 last_idx 설정 (급회전 방지)
    try:
        dmin, imin = float("inf"), 0
        for i in range(len(traj_x)):
            d = getDistance([current_x, current_y], traj.getPoint(i))
            if d < dmin:
                dmin, imin = d, i
        traj.last_idx = imin
        rospy.loginfo("Start from nearest waypoint idx=%d (dist=%.2f m)", imin, dmin)
    except Exception as e:
        rospy.logwarn("Nearest index init failed: %s", e)

    while (not rospy.is_shutdown()) and (getDistance([current_x, current_y], goal) > 1):


        target_point = traj.getTargetPoint([current_x, current_y])

        ###  waitpoint에 도착했을때 멈추기 cnt가 10될때 까지 속도를 0으로 설정해서 멈추게 하기
        ###  + 후진포인트와 wait포인트가 겹칠때 멈춤먼저 하기위해 if문에 멈춤 코드 배치
        if (traj.last_idx in wait_point) and (cnt<=10):
            velocity.linear.x = 0
            cnt=cnt+1

        #######후진을해야하는 웨이포인트에 해당할때 후진모드로 설정, 조향을 반대로?
        elif traj.last_idx in back_point:
            cnt=0

            ###차량의 헤딩방향을 정반대로 만들어주는 코드
            if current_yaw>0:
                back_mode_yaw=current_yaw-math.pi
            else:
                back_mode_yaw=current_yaw+math.pi

            # [MOD] 0 나눗셈 방지용 거리 하한
            dist_tp = max(getDistance([current_x,current_y], target_point), 1e-3)  # [MOD]

            alpha = math.atan2(
                (target_point[1] - current_y) * math.cos(back_mode_yaw) - (target_point[0] - current_x) * math.sin(
                    back_mode_yaw),
                (target_point[0] - current_x) * math.cos(back_mode_yaw) + (target_point[1] - current_y) * math.sin(
                    back_mode_yaw))

            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), dist_tp))*(-1)  # [MOD]
            steering_command = steering_angle * kp_angular

            velocity.linear.x = -55
            velocity.angular.z = steering_command


        else:
            cnt=0

            # [MOD] 0 나눗셈 방지용 거리 하한
            dist_tp = max(getDistance([current_x,current_y], target_point), 1e-3)  # [MOD]

            alpha = math.atan2(
                (target_point[1] - current_y) * math.cos(current_yaw) - (target_point[0] - current_x) * math.sin(
                    current_yaw),
                (target_point[0] - current_x) * math.cos(current_yaw) + (target_point[1] - current_y) * math.sin(
                    current_yaw))

            steering_angle = math.degrees(math.atan2(2 * wheelbase * math.sin(alpha), dist_tp))  # [MOD]
            steering_command = steering_angle * kp_angular

            velocity.linear.x = 55
            velocity.angular.z = steering_command

        velocity_pub.publish(velocity)
        rate.sleep()

    # [MOD] 목표 도달 후 안전 정지
    rospy.loginfo("Goal reached. publish zero cmd and exit.")  # [MOD]
    velocity.linear.x = 0.0
    velocity.angular.z = 0.0
    for _ in range(10):
        velocity_pub.publish(velocity)
        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

