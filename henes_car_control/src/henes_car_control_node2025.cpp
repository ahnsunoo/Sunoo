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

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

float Left_MAX,Right_MAX;

float T_steering_angle ;
int   T_motor_speed ;

float W_steering_angle;
int   W_motor_speed=0 ;

int warning_flag = 0;
int traffic_state = 0;
int mode = 4;

// === STOP TIMER ADDED ===
ros::Time stop_start_time;
bool stop_timer_active = false;
int  stop_reason_mode = 0;     // 1: emergency, 2: traffic
const double STOP_HOLD_SEC = 3.0;
// ========================

// === EMERGENCY REVERSE PULSE (NEW) ===
bool       rev_pulse_active = false;         // 후진 펄스 진행 중 여부
ros::Time  rev_pulse_start;                  // 펄스 시작 시각
const double REV_PULSE_SEC = 0.5;            // 펄스 길이(초): "잠깐" 후진하는 시간
// =====================================

/**
 * @brief /emergency_stop 토픽 콜백
 */
void EmergencyStopCallback(const std_msgs::Bool& msg)
{
    int new_flag = static_cast<int>(msg.data);

    // 라이징 엣지(0->1)에서만 후진 펄스 트리거
    if (new_flag && !warning_flag) {
        rev_pulse_active = true;
        rev_pulse_start  = ros::Time::now();
        printf("Emergency rising edge → start reverse pulse for %.1fs\n", REV_PULSE_SEC);
    }

    warning_flag = new_flag;
    printf("Emergency flag updated: %d\n", warning_flag);
}

/**
 * @brief /red_sign 토픽 콜백
 */
void Traffic_object_callback(const std_msgs::Bool& msg)
{
    traffic_state = static_cast<int>(msg.data);
    printf("Traffic flag updated: %d\n", traffic_state);
}

void update_mode_logic()
{
    if (warning_flag) {
        mode = 1; // 긴급 정지
        printf("Mode set to 1 (Emergency Stop)\n");
    }
    else if (traffic_state) {
        mode = 2; // 신호등 정지
        printf("Mode set to 2 (Traffic Stop)\n");
    }
    else {
        mode = 0; // 주행
        printf("Mode set to 0 (Driving)\n");
    }
}

/**
 * @brief /gps_cmd_vel 콜백
 */
void Way_Point_Car_Control_Callback2(const geometry_msgs::Twist& msg)
{
   W_steering_angle = (float)(msg.angular.z);
   W_motor_speed = (int)msg.linear.x;

   if(W_motor_speed>255)   W_motor_speed = 255;
   if(W_motor_speed<-255)  W_motor_speed = -255;

   if(mode == 0)
   {
      printf("Waypoint Driving Start %d\n" , W_motor_speed);
      mode = 3; // 자율주행 모드
   }
}

ros::Time last_button_press_time;
/**
 * @brief /joy 콜백
 */
void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    ros::Time current_time = ros::Time::now();
    ros::Duration time_since_last_press = current_time - last_button_press_time;

    if (joy_msg->buttons[7] == 1 && time_since_last_press.toSec() >= 1.0)
    {
        printf("toggle Button pressed\n");
        if (mode == 4)      mode = 0; // 수동→자율
        else                mode = 4; // 그 외→수동
        last_button_press_time = current_time;
    }
}

/**
 * @brief /cmd_vel 콜백
 */
void Teleop_Callback(const geometry_msgs::Twist& msg)
{
    T_steering_angle = (float)(msg.angular.z);
    T_motor_speed = (float)msg.linear.x;

    if(T_motor_speed>=100)   T_motor_speed = 100;
    if(T_motor_speed<=-100)  T_motor_speed = -100;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "henes_car_control_node");
  ros::NodeHandle n;

  std::string cmd_vel_topic = "cmd_vel";
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);

  geometry_msgs::Twist teleop_cmd_vel_data;

  ros::Subscriber teleop = n.subscribe("/cmd_vel", 10, &Teleop_Callback);
  ros::Subscriber joy_sub = n.subscribe("/joy", 10, &JoyCallback);
  ros::Subscriber sub_traffic   = n.subscribe("/red_sign", 10, Traffic_object_callback);
  ros::Subscriber sub_emergency = n.subscribe("/emergency_stop", 10, EmergencyStopCallback);
  ros::Publisher  teleop_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/teleop_cmd_vel", 20);
  ros::Subscriber Way_point = n.subscribe("/gps_cmd_vel",1, &Way_Point_Car_Control_Callback2);

  ros::Rate loop_rate(10);
  int count = 0;

  ros::Duration(1).sleep();

  while (ros::ok())
  {
    ros::spinOnce();

    // 수동 모드(4) 아닐 때만 모드 업데이트
    if (mode != 4)
    {
      update_mode_logic();
    }

    // === STOP TIMER: 비정지 모드에서 타이머/이유 리셋 ===
    if (mode != 1 && mode != 2) {
        stop_timer_active = false;
        stop_reason_mode = 0;
    }
    // ====================================================

    if (mode == 1) // ==== Emergency Stop ====
    {
      // 3초 정지 보장용 타이머
      if (!stop_timer_active || stop_reason_mode != mode) {
          stop_timer_active = true;
          stop_reason_mode = mode;
          stop_start_time = ros::Time::now();
          printf("<---- Stop timer started (EMERGENCY) ---->\n");
      }
      ros::Duration elapsed = ros::Time::now() - stop_start_time;

      // ----- NEW: 후진 펄스 처리 -----
      bool did_reverse = false;
      if (rev_pulse_active) {
          ros::Duration rev_elapsed = ros::Time::now() - rev_pulse_start;
          if (rev_elapsed.toSec() < REV_PULSE_SEC) {
              teleop_cmd_vel_data.linear.x = -20;  // 잠깐 후진
              did_reverse = true;
          } else {
              rev_pulse_active = false;           // 펄스 종료
          }
      }
      if (!did_reverse) {
          teleop_cmd_vel_data.linear.x = 0;       // 펄스 없으면 정지 유지
      }
      // --------------------------------

      teleop_cmd_vel_data.angular.z = 0;          // 조향 0
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);

      printf("[EMERGENCY] STOP (%.2f/%.1f s)%s\n",
             elapsed.toSec(), STOP_HOLD_SEC, did_reverse ? " [reverse pulse]" : "");
      // 원인(warning_flag)이 살아있으면 계속 정지. 해제되면 윗단에서 주행모드 복귀.
    }
    else if (mode == 2) // ==== Traffic Stop ====
    {
      if (!stop_timer_active || stop_reason_mode != mode) {
          stop_timer_active = true;
          stop_reason_mode = mode;
          stop_start_time = ros::Time::now();
          printf("<---- Stop timer started (TRAFFIC) ---->\n");
      }
      ros::Duration elapsed = ros::Time::now() - stop_start_time;

      teleop_cmd_vel_data.linear.x = 0;           // 신호 정지는 후진 없이 정지 유지
      teleop_cmd_vel_data.angular.z = 0;
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);

      printf("[TRAFFIC] STOP (%.2f/%.1f s)\n", elapsed.toSec(), STOP_HOLD_SEC);
    }
    else if (mode == 4) // ==== Teleop ====
    {
      printf("<--------Teleop-------->\n");
      teleop_cmd_vel_data.angular.z = T_steering_angle;
      teleop_cmd_vel_data.linear.x  = T_motor_speed;
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
      printf("T_motor_speed %d\n",T_motor_speed);
      printf("T_steering_angle %f\n",T_steering_angle);
    }
    else if ((mode == 3) || (mode == 0)) // ==== Waypoint / Driving ====
    {
      teleop_cmd_vel_data.angular.z = W_steering_angle;
      teleop_cmd_vel_data.linear.x  = W_motor_speed;
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
    }

    loop_rate.sleep();
    ++count;
  }

  W_motor_speed = 0;
  return 0;
}

