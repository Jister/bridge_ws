#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "mavros_extras/SonarDistance.h"
#include "sonar/Sonar_raw.h"
#include "lidar_lite_ros/Lidarlite.h"
#include "leddar_one/Leddar.h"

class DataHandle
{
public: 
	DataHandle();
private:
	ros::NodeHandle n;
	ros::Subscriber ground_distance_sub;
	ros::Subscriber bridge_distance_sub;
	ros::Publisher data_pub;

	void ground_distance_Callback(const leddar_one::Leddar &msg);
	void bridge_distance_Callback(const lidar_lite_ros::Lidarlite msg);
};

DataHandle::DataHandle()
{
	ground_distance_sub = n.subscribe("/ground_distance", 1, &FindContour::ground_distance_Callback,this);
	bridge_distance_sub = n.subscribe("/lidar_distance", 1, &FindContour::bridge_distance_Callback,this);
	data_pub = n.advertise<mavros_extras::SonarDistance>("/sonar_send",1);
}

void DataHandle::ground_distance_Callback(const leddar_one::Leddar &msg)
{
	mavros_extras::SonarDistance sonar_msg;
	sonar_msg.sonar_down = msg.distance.data;
	data_pub.publish(sonar_msg);
}

void DataHandle::bridge_distance_Callback(const lidar_lite_ros::Lidarlite msg)
{
	mavros_extras::SonarDistance sonar_msg;
	sonar_msg.sonar_up = msg.distance;
	data_pub.publish(sonar_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_handle");
	DataHandle DataHandle;
	ros::spin();
}
