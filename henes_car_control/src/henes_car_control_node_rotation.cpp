#define DEBUG 0
#define DEBUG_ROS_INFO 1 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
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
#define Right_MAX -30
#define Left_MAX  30   //고쳤으면 말을 해라

//Steering angle and motor speed define
int T_steering_angle ;
int T_motor_speed ;
float L_steering_angle ;
int L_motor_speed ;
int T_steering_angle_old ;
int T_motor_speed_old ;
int W_motor_speed ;
float W_steering_angle;

int Tunnel_motor_speed ;
float Tunnel_steering_angle ;

// warning value define <HJ>
int warning_flag;
int Tunnel_flag;
int driving_flag;
// int Lane_flags;


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

const char* traffic_obj;
int traffic_prob;
double Obj;


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
	
	 char buf[8];
  /*
   *   ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    */        
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      tf2::Matrix3x3 m(q);     
            
      m.getRPY(roll, pitch, yaw);
      yaw = yaw - DEG2RAD(imu_offset_angle);
      roll_d  = RAD2DEG(roll);
      pitch_d = RAD2DEG(pitch);
      yaw_d   = RAD2DEG(yaw);        
      //imu_read_flag_start ++;
            
}


void Teleop_Callback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
   
   T_steering_angle = (int)(msg.angular.z) ;
   
   if(T_steering_angle >= Left_MAX )   T_steering_angle = Left_MAX ;
   if(T_steering_angle <= Right_MAX )  T_steering_angle = Right_MAX ;
   
   T_motor_speed = (int)msg.linear.x;
   if(T_motor_speed>=100)   T_motor_speed = 100;
   if(T_motor_speed<=-100)  T_motor_speed = -100; 

   mode = 3;
}

//진석이 lane detection
void Lane_Detection_Callback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
   
   L_steering_angle = (int)(msg.angular.z) ;
   
   
   if(L_steering_angle >= Left_MAX )   L_steering_angle = Left_MAX ;
   if(L_steering_angle <= Right_MAX )  L_steering_angle = Right_MAX ;
   
   L_motor_speed = (int)msg.linear.x;
   if(L_motor_speed>=50)   L_motor_speed = 50;
   if(L_motor_speed<=-50)  L_motor_speed = -50; 


  //  Lane_flags = (int)(msg.linear.y);
   
   if(mode == 0 and warning_flag == 0 and driving_flag == 3) 
   {
    printf("Lane_Detection Start %d\n" , L_motor_speed);
    mode = 1;
   }
}

//<<--------------민재 waypoint car steer control----------------->>
void Way_Point_Car_Control_Callback2(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
   
   W_steering_angle = (int)(msg.angular.z) ;
   
   if(W_steering_angle >= Left_MAX )   W_steering_angle = Left_MAX ;
   if(W_steering_angle <= Right_MAX )  W_steering_angle = Right_MAX ;
   
   W_motor_speed = (int)msg.linear.x;
   if(W_motor_speed>=50)   W_motor_speed = 50;
   if(W_motor_speed<=-50)  W_motor_speed = -50; 

    if(mode == 0 and warning_flag == 0 and driving_flag == 1) 
    {
      printf("Waypoint Driving Start %d\n" , L_motor_speed);
      mode = 2;
    }
}

void Traffic_object_callback(const std_msgs::String& msg)
{
	traffic_obj = msg.data.c_str();
  ROS_INFO("traffic_object : %s",traffic_obj); 
  std::istringstream(traffic_obj) >> Obj;
  ROS_INFO("traffic_object : %f",Obj); 
  
}

void Traffic_probabilty_callback(const std_msgs::Int32& msg)
{
	traffic_prob = (int)(msg.data);
  ROS_INFO("traffic_probability : %d",traffic_prob);
  
}

//<---------------현지 lidar warning ------------------>//
void ScanCallback(const std_msgs::Int32& msg)
{
    warning_flag = (int)(msg.data);
    //ROS_INFO("Warning Flags: %d", warning_flag);
    if( warning_flag == 1 and driving_flag != 2) mode = 5;
    if( warning_flag == 2 and driving_flag != 2) mode = 6;
}

void Tunnel_Control_Callback(const geometry_msgs::Twist& msg)
{
    Tunnel_steering_angle = (float)(msg.angular.z) ;
   
   if(Tunnel_steering_angle >= Left_MAX )   Tunnel_steering_angle = Left_MAX ;
   if(Tunnel_steering_angle <= Right_MAX )  Tunnel_steering_angle = Right_MAX ;
   
   Tunnel_motor_speed = (int)msg.linear.x;
   if(Tunnel_motor_speed>=50)   Tunnel_motor_speed = 50;
   if(Tunnel_motor_speed<=-50)  Tunnel_motor_speed = -50; 

   if(mode == 0 and driving_flag == 2 and Tunnel_flag == 1) 
    {
      printf("Tunnel Driving Start \n");
      mode = 4;
    }
}

void Tunnel_Flags(const std_msgs::Int32& msg)
{
    Tunnel_flag = (int)(msg.data);
}

void Driving_Flags(const std_msgs::Int32& msg)
{
    driving_flag = (int)(msg.data);
}

void Timer(double a)
{
  double endTime = (unsigned)time(NULL);
  endTime += a;
  while(1)
  {
    int startTime = (unsigned)time(NULL);
    printf("%f초\n",endTime - startTime);
    if (endTime - startTime == 0)
    {
      printf("종료되었습니다.\n");
      return;
    }
  }
}

int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "Henes_Car_Control");

  ros::NodeHandle n;
  
  std::string cmd_vel_topic = "cmd_vel";
  std::string odom_pub_topic = "/odom";
  std::string imu_topic = "handsfree/imu";
   
  
  /*other*/
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  ros::param::get("~imu_topic", imu_topic); 
  ros::param::get("/imu_offest",  imu_offset_angle);
  
  
  std_msgs::String msg;
  std_msgs::Int16 steerangle;
  std_msgs::Int16 carspeed;
  
    
  geometry_msgs::Twist teleop_cmd_vel_data;
  
  ros::Subscriber teleop = n.subscribe("/cmd_vel", 10, &Teleop_Callback);
  ros::Subscriber subIMU = n.subscribe("/handsfree/imu", 20, &imuCallback);  // imu data susscribe
  ros::Subscriber traffic_Object = n.subscribe("/darknet_ros/detection_result_object", 1, &Traffic_object_callback);
  ros::Subscriber traffic_probabilty = n.subscribe("/darknet_ros/detection_result_probability", 1, &Traffic_probabilty_callback);
  
  //ROS_INFO("traffic_Object : %s | traffic_probabilty : %d", traffic_Object.data, traffic_probabilty.data );  
  // <<<<------------ 진석 made camera subscriber lane detection------------->>>>>>>
  ros::Subscriber Lane_Detection = n.subscribe("/henes_lane",1, &Lane_Detection_Callback);
  // <<<<------------ 민재 made gps subscriber waypoint ------------->>>>>>>
  ros::Subscriber Way_point = n.subscribe("/gps_cmd_vel",1, &Way_Point_Car_Control_Callback2);
  ros::Subscriber Driving_flag_sub = n.subscribe("/waypoint_flag", 1, &Driving_Flags);
  ///<<<<------------ 현지 made teleop subscriber ------------->>>>>>>
  ros::Publisher teleop_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/teleop_cmd_vel", 20);
  ///<<<<------------ 현지 made 긴급정거 subscriber ------------->>>>>>>
  ros::Subscriber Warning_sub = n.subscribe("/warning_flags", 1, &ScanCallback);
    ///<<<<------------ 현지 made tunnel control subscriber ------------->>>>>>>
  ros::Subscriber Tunnel_sub = n.subscribe("/tunnel_control", 1, &Tunnel_Control_Callback);
  ros::Subscriber Tunnel_flag_sub = n.subscribe("/tunnel_flag", 1, &Tunnel_Flags);
 
  ros::Rate loop_rate(10);  // 10

  int count = 0;
  
  //double Obj = std::stod(traffic_obj);
  ros::Duration(1).sleep();   // 1 sec stop for safety
  
  while (ros::ok())
  { 
    // printf("\n %d \n", mode);
    //printf("Current Status : Mode %d\n", mode);
    /* traffic light 판단하는 if 문
    if(Obj)
    {
      //printf("traffic_object : %s \n",traffic_obj);
      //std::istringstream(traffic_obj) >> Obj;
      printf("Obj : %.0f\n",Obj);
      printf("Obj : %f\n",Obj);
      printf("확인을 위한 출력 : traffic lights detection 있을 때 \n");  
      if(Obj != 1300 and Obj != 1400 and Obj != 1405 and Obj != 1502)
      {
        printf("stop\n");
      }
      else {printf("GO\n");}
      Obj = 0;
    } */
    if(mode == 3)
    {
       teleop_cmd_vel_data.angular.z = T_steering_angle;
	     teleop_cmd_vel_data.linear.x  = T_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
       printf("T_motor_speed %d\n",T_motor_speed);
       printf("T_steering_angle %d\n",T_steering_angle);

       mode = 0;
    }

    if (mode == 1)
    {
      printf("<----------Lane_detection----------->\n");

      /*if(L_steering_angle != steering_angle_old) 
      {
       teleop_cmd_vel_data.angular.z = L_steering_angle;
	     teleop_cmd_vel_data.linear.x  = L_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
       printf("L_motor_speed %d\n",L_motor_speed);
       printf("L_steering_angle %f\n",L_steering_angle);
      }
    
      if(L_motor_speed != motor_speed_old)
      {    
       teleop_cmd_vel_data.angular.z = L_steering_angle;
	     teleop_cmd_vel_data.linear.x  = L_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
       printf("L_motor_speed %d\n",L_motor_speed);
       printf("L_steering_angle %f\n",L_steering_angle);
      }*/

      teleop_cmd_vel_data.angular.z = L_steering_angle;
	    teleop_cmd_vel_data.linear.x  = L_motor_speed;
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
      printf("L_motor_speed %d\n",L_motor_speed);
      printf("L_steering_angle %f\n",L_steering_angle);
      
      //steering_angle_old = L_steering_angle;
      //motor_speed_old = L_motor_speed ; 

      //if(Lane_flags == 1) mode = 0;

      mode = 0;
    }

    if (mode == 2)
    {
      printf("<--------Waypoint Car control---------->\n");
            
      /*if(W_steering_angle != steering_angle_old) 
      {
       teleop_cmd_vel_data.angular.z = W_steering_angle;
	     teleop_cmd_vel_data.linear.x  = W_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
       printf("W_motor_speed %d\n",W_motor_speed);
       printf("W_steering_angle %f\n",W_steering_angle);
      }
    
      if(W_motor_speed != motor_speed_old)
      {    
       teleop_cmd_vel_data.angular.z = W_steering_angle;
	     teleop_cmd_vel_data.linear.x  = W_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
       printf("W_motor_speed %d\n",W_motor_speed);
       printf("W_steering_angle %f\n",W_steering_angle);
      } */

       teleop_cmd_vel_data.angular.z = W_steering_angle;
	     teleop_cmd_vel_data.linear.x  = W_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
       printf("W_motor_speed %d\n",W_motor_speed);
       printf("W_steering_angle %f\n",W_steering_angle);

      //steering_angle_old = W_steering_angle;
      //motor_speed_old = W_motor_speed ; 

      mode = 0;
    }

    if (mode == 4)
    {
      printf("<--------Tunnel Car control---------->\n");
      //printf("steering_angle_old %d\n",steering_angle_old);
      //printf("Tunnel_motor_speed %d\n",Tunnel_motor_speed);
      //printf("Tunnel_steering_angle %f\n",Tunnel_steering_angle);
      /*if(Tunnel_steering_angle != steering_angle_old) 
      {
       teleop_cmd_vel_data.angular.z = Tunnel_steering_angle;
	     teleop_cmd_vel_data.linear.x  = Tunnel_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
       printf("Tunnel_motor_speed %d\n",Tunnel_motor_speed);
       printf("Tunnel_steering_angle %f\n",Tunnel_steering_angle);
      }
    
      if(Tunnel_motor_speed != motor_speed_old)
      {    
       teleop_cmd_vel_data.angular.z = Tunnel_steering_angle;
	     teleop_cmd_vel_data.linear.x  = Tunnel_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
       printf("Tunnel_motor_speed %d\n",Tunnel_motor_speed);
       printf("Tunnel_steering_angle %f\n",Tunnel_steering_angle);
      } 
      */
       teleop_cmd_vel_data.angular.z = Tunnel_steering_angle;
	     teleop_cmd_vel_data.linear.x  = Tunnel_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
       printf("Tunnel_motor_speed %d\n",Tunnel_motor_speed);
       printf("Tunnel_steering_angle %f\n",Tunnel_steering_angle);
   
      //steering_angle_old = Tunnel_steering_angle;
      //motor_speed_old = Tunnel_motor_speed ; 

      mode = 0;
    }

    if(mode == 5)
    {
      printf("<-------------------Warning-------------------->\n");
      // teleop_cmd_vel_data.angular.z = 0;
      teleop_cmd_vel_data.linear.x  = 0;
      teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
       
      mode = 0;
    }

    if(mode==6)
    {
      time_t start = time(NULL);
      time_t end = time(NULL);
      printf("Start static obstacle");

      // printf("Start 1 \n");
      // teleop_cmd_vel_data.angular.z = 30;
      // teleop_cmd_vel_data.linear.x = 20;
      // teleop_cmd_vel_pub.publish(teleop_cmd_vel_data); 
      // ros::Duration(2).sleep();   // 1 sec stop for safety

      // printf("Start 2 \n");
      // teleop_cmd_vel_data.angular.z = 0;
      // teleop_cmd_vel_data.linear.x = 20;
      // teleop_cmd_vel_pub.publish(teleop_cmd_vel_data); 
      // ros::Duration(2).sleep();

      // printf("========== END ==========");
      // mode = 0;

      while((double)(end-start) < 7)
      {
        end = time(NULL);
        printf("%f\n",(double)(end-start));

        if (end-start < 3)
        {
          printf("Start 1 \n");
          teleop_cmd_vel_data.angular.z = 30;
          teleop_cmd_vel_data.linear.x = 20;
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);          
        }
        else if (end-start < 5)
        {  
          printf("Start 2 \n");  
          teleop_cmd_vel_data.angular.z = 0;
          teleop_cmd_vel_data.linear.x = 20;
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
        }
        else if (end-start < 7)
        {
          printf("Start 3 \n");  
          teleop_cmd_vel_data.angular.z = -30;
          teleop_cmd_vel_data.linear.x = 20;
          teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
        }
        if (end-start == 7)
        {
          printf("========== END ==========");
          mode = 0;
        }
      }
      
    }
    /*
    else {

    //printf("확인을 위한 출력 : traffic lights detection 없을 때 \n");    
    //printf("Speed : %3d | Steering : %2d \n", W_motor_speed, W_steering_angle ); 
    if(W_steering_angle != steering_angle_old) 
    {
       teleop_cmd_vel_data.angular.z = W_steering_angle;
	     teleop_cmd_vel_data.linear.x  = W_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
    }
    
    if(W_motor_speed != motor_speed_old)
    {    
       teleop_cmd_vel_data.angular.z = W_steering_angle;
	   teleop_cmd_vel_data.linear.x  = W_motor_speed;
       teleop_cmd_vel_pub.publish(teleop_cmd_vel_data) ;
    } 
   
    steering_angle_old = W_steering_angle;
    motor_speed_old = W_motor_speed ; 
    
	   
    steerangle.data = W_steering_angle;
    carspeed.data = W_motor_speed;
    //carspeed.data = motor_speed;
     

    car_control_pub1.publish(steerangle);
    car_control_pub2.publish(carspeed);
    } */

    
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }

  W_motor_speed = 0;
  
  return 0;
}




