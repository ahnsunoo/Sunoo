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
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include <string>
#include <iostream>
#include<time.h>


int Lane_mode = 0;

void FixCallback(const sensor_msgs::NavSatFix& msg)
{
  if (msg.position_covariance[0] > 1.0)
  {
    Lane_mode = 1;
  }
  else
  {
    Lane_mode = 0;
  }
}


int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "Henes_Car_Control");

  ros::NodeHandle n;
  
  std_msgs::String msg;
  std_msgs::Int16 msgs;
  std_msgs::Int16 carspeed;    
  geometry_msgs::Twist teleop_cmd_vel_data;
  
  ros::Subscriber gps_accuracy = n.subscribe("/ublox_gps/fix", 1, &FixCallback); 
  
  
  ros::Rate loop_rate(10);  // 10

  int count = 0;
  
  ros::Duration(1).sleep();   // 1 sec stop for safety
  
  while (ros::ok())
  {
    if (Lane_mode == 1)
    {
      printf("Lane입니다. 이경민.\n");
    }
    else
    {
      printf("GPS입니다.\n");
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}





