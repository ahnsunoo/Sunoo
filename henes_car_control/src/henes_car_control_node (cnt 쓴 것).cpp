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

// unit m
#define Sonar_Detect_MAX_Range 3.0
#define Sonar_Obstacle_Range   1.0
#define Lidar_Obstacle_Range   1.0

// Steering Control
#define Neutral_Angle_Offset  0
#define Right_MAX -45
#define Left_MAX  45   //고쳤으면 말을 해라

// timer
#define EndTime1 3.0
#define EndTime2 0.2
#define EndTime3 2.0

#define Slope_Motor_Speed 0.0


int T_steering_angle ;
int T_motor_speed ;
float L_steering_angle ;
int L_motor_speed ;
int T_steering_angle_old ;
int T_motor_speed_old ;
int W_motor_speed ;
float W_steering_angle;

int warning_flag = 0;
int traffic_state = 0;
int Tunnel_flag;
int driving_flag;


int Base_Speed = 40;
int steering_angle_old = 0;
int motor_speed_old = 0;


int mode = 0;
double roll,pitch,yaw,yaw_old;
double roll_d,pitch_d,yaw_d,yaw_d_old;
double delta_yaw_d, delta_yaw;
int  imu_read_flag_start = 0;
bool imu_init_angle_flag = 0;
float init_yaw_angle = 0.0;
float init_yaw_angle_d = 0.0;
float imu_offset_angle = 0;  // degree
int imu_mode = 0;
const char* traffic_obj;
int traffic_prob;
double Obj;

double startTime = (unsigned)time(NULL);
double currentTime = (unsigned)time(NULL);
double totalTime = (unsigned)time(NULL);
int cnt = 0;


void EmergencyStopCallback(const std_msgs::Bool& msg) // 사람 감지 
{
    warning_flag = (int)(msg.data);
    if (warning_flag)
    {
      printf("emergency detection\n");
      mode = 1;
    }
    else
    {
      mode = 0;
    }
}


void Traffic_object_callback(const std_msgs::Bool& msg) // 신호등 감지
{
    traffic_state = (int)(msg.data);
    if (traffic_state)
    {
        printf("traffic_detection\n");
        mode = 2;
    }
    else
    {
      mode=0;
    }
}

//<<--------------waypoint car steer control----------------->>
void Way_Point_Car_Control_Callback2(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
   
   W_steering_angle = (int)(msg.angular.z) ;
   
  //  if(W_steering_angle >= Left_MAX )   W_steering_angle = Left_MAX ;
  //  if(W_steering_angle <= Right_MAX )  W_steering_angle = Right_MAX ;
   
   W_motor_speed = (int)msg.linear.x;
   if(W_motor_speed>=255)   W_motor_speed = 255;
   if(W_motor_speed<=-255)  W_motor_speed = -255; 

    if(mode == 0) 
    {
      printf("Waypoint Driving Start %d\n" , W_motor_speed);
      mode = 3;
    } 
}

ros::Time last_button_press_time;
void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    ros::Time current_time = ros::Time::now();

    ros::Duration time_since_last_press = current_time - last_button_press_time;

    if (joy_msg->buttons[7] == 1 && time_since_last_press.toSec() >= 1.0) 
    {
        printf("toggle Button pressed\n");
        if (mode == 4)
        {
            mode = 0;
        }
        else
        {
            mode = 4;
        } 

        last_button_press_time = current_time;
    }
}

void Teleop_Callback(const geometry_msgs::Twist& msg)
{
    T_steering_angle = (int)(msg.angular.z) ;
   
  //  if(T_steering_angle >= Left_MAX )   T_steering_angle = Left_MAX ;
  //  if(T_steering_angle <= Right_MAX )  T_steering_angle = Right_MAX ;
   
   T_motor_speed = (int)msg.linear.x;
   if(T_motor_speed>=100)   T_motor_speed = 100;
   if(T_motor_speed<=-100)  T_motor_speed = -100; 
}



void PitchCallback(const std_msgs::Int32& msg)
{
  if (msg.data == 1)
  {
    imu_mode = 1;
  }
  else if (msg.data == 2)
  {
    imu_mode = -1;
  }
  else
  {
    imu_mode = 0;
  }
}



int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "henes_car_control_node");

  ros::NodeHandle n;
  
  std::string cmd_vel_topic = "cmd_vel";
  // std::string odom_pub_topic = "/odom";
  // std::string imu_topic = "handsfree/imu";
   
  
  /*other*/
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);
  // ros::param::get("~odom_pub_topic", odom_pub_topic);
  // ros::param::get("~imu_topic", imu_topic); 
  // ros::param::get("/imu_offest",  imu_offset_angle);
  
  
  std_msgs::String msg;
  std_msgs::Int16 msgs;
  std_msgs::Int16 carspeed;    
  geometry_msgs::Twist teleop_cmd_vel_data;
  
  ros::Subscriber teleop = n.subscribe("/cmd_vel", 10, &Teleop_Callback);
  ros::Subscriber joy_sub = n.subscribe("/joy", 10, &JoyCallback);
  ros::Subscriber traffic_Object = n.subscribe("/red_sign", 1, &Traffic_object_callback); // detection traffic
  ros::Subscriber emergency_Object = n.subscribe("/person_signal", 1, &EmergencyStopCallback); // detection person
  ros::Subscriber imuMode = n.subscribe("/imu_mode", 1, &PitchCallback); // imu pitch callback
  ros::Publisher teleop_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/teleop_cmd_vel", 20);
  

  // ros::Publisher mode_pub = n.advertise<std_msgs::Int16>("/mode", 20);
  
  ros::Subscriber Way_point = n.subscribe("/gps_cmd_vel",1, &Way_Point_Car_Control_Callback2);
  // ros::Subscriber Driving_flag_sub = n.subscribe("/waypoint_flag", 1, &Driving_Flags);
  
  ros::Rate loop_rate(10);  // 10

  int count = 0;
  
  //double Obj = std::stod(traffic_obj);
  ros::Duration(1).sleep();   // 1 sec stop for safety
  
  while (ros::ok())
  {
    if ((mode == 1)||(mode == 2))
    {
      //  mode_pub.publish(mode);
       printf("<--------------Emergency/Traffic-------------->\n");
       teleop_cmd_vel_data.linear.x = 0;
       teleop_cmd_vel_data.angular.z = 0;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
    
    }

    if (mode == 4)
    {
      printf("<--------Teleop-------->\n");
      teleop_cmd_vel_data.angular.z = T_steering_angle;
	    teleop_cmd_vel_data.linear.x  = T_motor_speed;
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;


      printf("T_motor_speed %d\n",T_motor_speed);
      printf("T_steering_angle %d\n",T_steering_angle);
    }

    if ((mode == 3) || (mode == 0))
    {
      if (imu_mode == 0) 
      {
        // printf("<--------original velocity-------->\n");
        teleop_cmd_vel_data.angular.z = W_steering_angle;
        teleop_cmd_vel_data.linear.x  = W_motor_speed;
        teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
        // printf("L_motor_speed %d\n",L_motor_speed);
        // printf("L_steering_angle %d\n",L_steering_angle);
      }

      if (imu_mode == 1) // go
      {
        cnt += 1;

        if (cnt <= 50)
        {
          printf("============ up ============\n");
          teleop_cmd_vel_data.angular.z = W_steering_angle;
          teleop_cmd_vel_data.linear.x  = 110;
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
        }

        else if ((cnt > 50) && (cnt <= 50+45)) 
        {
          printf("================================ Stop ================================\n");
          teleop_cmd_vel_data.angular.z = W_steering_angle;
          teleop_cmd_vel_data.linear.x  = 60;
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
        }

        else
        {
          printf("============ up restart ============\n");
          teleop_cmd_vel_data.angular.z = W_steering_angle;
          teleop_cmd_vel_data.linear.x  = 110;
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
        }
      }

      if (imu_mode == -1)
      {
        printf("============ down ============\n");
          // printf("<--------original velocity-------->\n");
          teleop_cmd_vel_data.angular.z = W_steering_angle;
          teleop_cmd_vel_data.linear.x  = -20;
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
          // printf("L_motor_speed %d\n",L_motor_speed);
          // printf("L_steering_angle %d\n",L_steering_angle);
      }
    }


    
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }

  W_motor_speed = 0;
  
  return 0;
}




