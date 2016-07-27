#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "mavros_extras/LaserDistance.h"
#include <math.h>

mavros_extras::LaserDistance pos;
void scanCallback(const sensor_msgs::LaserScan laser);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_min_distance");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
  ros::Publisher pub = n.advertise<mavros_extras::LaserDistance>("/laser_send", 2);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void scanCallback(const sensor_msgs::LaserScan laser)
{
  float min_distance;
  int angle;

  min_distance = 60;
  angle = 0;
  for(int i=0; i<laser.ranges.size(); i++)
  {
    if(laser.ranges[i] > 0.5)
    {
    	if(laser.ranges[i]<min_distance)
	{
  	      min_distance = laser.ranges[i];
	      angle=i*0.25;
	}
    }
	    
  }
  pos.min_distance = min_distance*100;
  pos.angle = angle-45;

} 

