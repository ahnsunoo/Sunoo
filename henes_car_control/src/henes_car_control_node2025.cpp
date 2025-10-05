#include "ros/ros.h"
#include "unistd.h" // for sleep
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Joy.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include <string>
#include <iostream>
#include <time.h>

// 라디안을 도로 변환하는 매크로 정의
#define RAD2DEG(x) ((x)*180./M_PI)
// 도를 라디안으로 변환하는 매크로 정의
#define DEG2RAD(x) ((x)/180.*M_PI)


float Left_MAX,Right_MAX; // 좌우 최대 조향각 변수 (코드 내에서 사용되지 않음, 주석 처리된 부분에서 사용될 예정이었을 것으로 추정)


// 조이스틱 및 웨이포인트로부터 받은 주행 속도와 조향각 변수 선언
float T_steering_angle ; // Teleop(조이스틱) 모드에서의 조향각
int T_motor_speed ;      // Teleop(조이스틱) 모드에서의 모터 속도

float W_steering_angle; // Waypoint(자율주행) 모드에서의 조향각
int W_motor_speed=0 ;     // Waypoint(자율주행) 모드에서의 모터 속도


// 장애물 및 신호 감지를 위한 플래그 변수
int warning_flag = 0;   // 장애물 감지 플래그 (0: 없음, 1: 감지)
int traffic_state = 0;  // 신호등 감지 플래그 (0: 없음, 1: 감지)
int mode = 4;           // 현재 차량의 주행 모드 (초기값: 4=수동 모드)



/**
 * @brief /person_signal 토픽을 구독하여 장애물 감지 플래그를 업데이트하는 콜백 함수
 * @param msg std_msgs::Bool 메시지. true(1)이면 장애물 감지
 * @subscribes /person_signal (std_msgs::Bool)
 */
void EmergencyStopCallback(const std_msgs::Bool& msg)
{
    // 콜백 함수는 자신의 상태만 업데이트
    warning_flag = static_cast<int>(msg.data);
    printf("Emergency flag updated: %d\n", warning_flag);

}

/**
 * @brief /red_sign 토픽을 구독하여 신호등 감지 플래그를 업데이트하는 콜백 함수
 * @param msg std_msgs::Bool 메시지. true(1)이면 신호등 감지
 * @subscribes /red_sign (std_msgs::Bool)
 */
void Traffic_object_callback(const std_msgs::Bool& msg)
{
    // 콜백 함수는 자신의 상태만 업데이트
    traffic_state = static_cast<int>(msg.data);
    printf("Traffic flag updated: %d\n", traffic_state);
}

/**
 * @brief 장애물 및 신호등 플래그에 따라 최종 주행 모드를 결정하는 함수
 * @details 이 함수는 메인 루프에서 호출되어 차량의 우선순위를 결정합니다.
 */
void update_mode_logic()
{
    // 단일 함수에서 모든 상태를 종합적으로 판단하여 mode 결정
    if (warning_flag) {
        mode = 1; // 장애물이 감지되면 긴급 정지 모드 (최우선 순위)
        printf("Mode set to 1 (Emergency Stop)\n");
    }
    else if (traffic_state) {
        mode = 2; // 신호등이 감지되면 신호등 정지 모드
        printf("Mode set to 2 (Traffic Stop)\n");
    }
    else {
        mode = 0; // 둘 다 아니면 주행 모드
        printf("Mode set to 0 (Driving)\n");
    }
}




/**
 * @brief /gps_cmd_vel 토픽을 구독하여 자율주행 모드의 속도와 조향각을 업데이트하는 콜백 함수
 * @param msg geometry_msgs::Twist 메시지. linear.x는 속도, angular.z는 조향각
 * @subscribes /gps_cmd_vel (geometry_msgs::Twist)
 */
void Way_Point_Car_Control_Callback2(const geometry_msgs::Twist& msg)
{
   W_steering_angle = (float)(msg.angular.z) ;

   // 주석 처리된 부분: 조향각의 최대/최소값을 제한하는 로직
   // if(W_steering_angle >= Left_MAX )   W_steering_angle = Left_MAX ;
   // if(W_steering_angle <= Right_MAX )  W_steering_angle = Right_MAX ;

   W_motor_speed = (int)msg.linear.x;

   // 모터 속도 최대/최소값 제한
   if(W_motor_speed>255)   W_motor_speed = 255;
   if(W_motor_speed<-255)  W_motor_speed = -255;

    // 모드 0(Driving) 상태에서 자율주행 데이터가 들어오면 모드를 3으로 변경
    if(mode == 0)
    {
      printf("Waypoint Driving Start %d\n" , W_motor_speed);
      mode = 3; // 자율주행 모드 (Driving)
    }
}


ros::Time last_button_press_time; // 마지막 조이스틱 버튼이 눌린 시간
/**
 * @brief /joy 토픽을 구독하여 조이스틱 버튼 입력을 처리하는 콜백 함수
 * @param joy_msg sensor_msgs::Joy::ConstPtr 메시지. buttons[7]가 눌렸는지 확인
 * @subscribes /joy (sensor_msgs::Joy)
 */
void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    ros::Time current_time = ros::Time::now();
    ros::Duration time_since_last_press = current_time - last_button_press_time;

    // 7번 버튼이 눌렸고, 마지막으로 누른지 1초가 지났다면
    if (joy_msg->buttons[7] == 1 && time_since_last_press.toSec() >= 1.0)
    {
        printf("toggle Button pressed\n");
        if (mode == 4)
        {
            mode = 0; // 수동 모드(4)에서 자율주행 모드(0)로 전환
        }
        else
        {
            mode = 4; // 다른 모드에서 수동 모드(4)로 전환
        }

        last_button_press_time = current_time; // 마지막으로 누른 시간 업데이트
    }
}

/**
 * @brief /cmd_vel 토픽을 구독하여 수동(Teleop) 모드의 속도와 조향각을 업데이트하는 콜백 함수
 * @param msg geometry_msgs::Twist 메시지. linear.x는 속도, angular.z는 조향각
 * @subscribes /cmd_vel (geometry_msgs::Twist)
 */
void Teleop_Callback(const geometry_msgs::Twist& msg)
{
    T_steering_angle = (float)(msg.angular.z) ;

    // 조향각의 최대/최소값 제한
    // if(T_steering_angle >= Left_MAX )   T_steering_angle = Left_MAX ;
    // if(T_steering_angle <= Right_MAX )  T_steering_angle = Right_MAX ;

    T_motor_speed = (float)msg.linear.x;

    // 모터 속도 최대/최소값 제한
    if(T_motor_speed>=100)   T_motor_speed = 100;
    if(T_motor_speed<=-100)  T_motor_speed = -100;
}

int main(int argc, char **argv)
{
  char buf[2];
  // ROS 노드 초기화. 노드 이름은 "henes_car_control_node"
  ros::init(argc, argv, "henes_car_control_node");
  ros::NodeHandle n;

  // ROS 파라미터 서버로부터 토픽 이름 가져오기 (기본값 설정)
  std::string cmd_vel_topic = "cmd_vel";
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);

  // 메시지 및 변수 선언
  std_msgs::String msg;
  std_msgs::Int16 msgs;
  std_msgs::Int16 carspeed;
  geometry_msgs::Twist teleop_cmd_vel_data;

  // ------------ ROS Subscriber 및 Publisher 정의 ------------
  // 조이스틱으로부터 속도와 조향을 받는 구독자
  ros::Subscriber teleop = n.subscribe("/cmd_vel", 10, &Teleop_Callback);

  // 수동/자동 모드 토글을 위한 조이스틱 버튼 구독자
  ros::Subscriber joy_sub = n.subscribe("/joy", 10, &JoyCallback);

  // 신호등 감지를 위한 구독자
  ros::Subscriber sub_traffic   = n.subscribe("/red_sign", 10, Traffic_object_callback);

  // 장애물 감지를 위한 구독자
  ros::Subscriber sub_emergency = n.subscribe("/emergency_stop", 10, EmergencyStopCallback);

  // 차량 제어를 위한 최종 신호를 보내는 Publisher
  // @publishes /teleop_cmd_vel (geometry_msgs::Twist)
  ros::Publisher teleop_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/teleop_cmd_vel", 20);

  // 자율주행 모드에서 조향 및 속도 데이터를 받는 구독자
  // @subscribes /gps_cmd_vel (geometry_msgs::Twist)
  ros::Subscriber Way_point = n.subscribe("/gps_cmd_vel",1, &Way_Point_Car_Control_Callback2);

  ros::Rate loop_rate(10);  // 루프 주기 설정 (10Hz)
  int count = 0;

  ros::Duration(1).sleep();   // 안전을 위해 시작 시 1초 대기

  // 메인 루프 시작
  while (ros::ok())
  {
    ros::spinOnce(); // 모든 콜백 함수들을 한 번씩 호출

    // 수동 모드(4)가 아닐 때만 update_mode_logic() 실행// 장애물/신호등 플래그를 바탕으로 주행 모드 업데이트
    if (mode != 4)
    {
      update_mode_logic();
    }

    if ((mode == 1)||(mode == 2))
    {
      // 모드가 긴급 정지(1) 또는 신호등 정지(2)일 때
      printf("<--------------Emergency/Traffic-------------->\n");
      teleop_cmd_vel_data.linear.x = 0; // 속도 0으로 설정
      teleop_cmd_vel_data.angular.z = 0; // 조향각 0으로 설정
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ; // 정지 명령 발행

      printf("stop\n");
    }
    else if (mode == 4)
    {
      // 모드가 수동(Teleop, 4)일 때
      printf("<--------Teleop-------->\n");
      teleop_cmd_vel_data.angular.z = T_steering_angle; // 조이스틱 조향각 사용
      teleop_cmd_vel_data.linear.x  = T_motor_speed;    // 조이스틱 속도 사용
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ; // 수동 제어 명령 발행
      printf("T_motor_speed %d\n",T_motor_speed);
      printf("T_steering_angle %f\n",T_steering_angle);
    }
    else if ((mode == 3) || (mode == 0))
    {
      // 모드가 자율주행(Driving, 3 또는 0)일 때
          teleop_cmd_vel_data.angular.z = W_steering_angle; // 웨이포인트 알고리즘 조향각 사용
          teleop_cmd_vel_data.linear.x  = W_motor_speed;    // 웨이포인트 알고리즘 속도 사용
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ; // 자율주행 제어 명령 발행
    }
    loop_rate.sleep(); // 다음 루프까지 대기
    ++count;

  }



  // ROS가 종료될 때 W_motor_speed를 0으로 초기화
  W_motor_speed = 0;







  return 0;
}