#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "mavros_extras/SonarDistance.h"
#include "sonar/Sonar_raw.h"
#include "lidar_lite_ros/Lidarlite.h"

float Front = 0;
float Back = 0;
float Left = 0;
float Right = 0;
float Up = 0;

void sonarCallback(const sonar::Sonar_raw sonar)
{
    Right=sonar.sonar_1;
    Left =sonar.sonar_2;
    Back=sonar.sonar_3;
}

void lidarCallback(const lidar_lite_ros::Lidarlite msg)
{
    Up = msg.distance;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_handle");
  ros::NodeHandle n;
  ros::Publisher sonar_pub = n.advertise<mavros_extras::SonarDistance>("sonar_send",10);
  ros::Subscriber sub1 = n.subscribe("sonar_data", 10, sonarCallback);
  ros::Subscriber sub2 = n.subscribe("lidar_distance", 10, lidarCallback);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    mavros_extras::SonarDistance sonar_msg;
    
    sonar_msg.sonar_front = 0;
    sonar_msg.sonar_behind = Back;
    sonar_msg.sonar_left = Left;
    sonar_msg.sonar_right = Right;
    sonar_msg.sonar_up = Up;
    
    sonar_pub.publish(sonar_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
