#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "laserscan/Laser.h"

laserscan::Laser pos;

void cloudCallback (const sensor_msgs::PointCloud cloud)
{
  float x = 0 ;
  float y = 0 ;
  int count = 0 ;

  for(int i=0;i<cloud.points.size();i++)
  {
    if((cloud.points[i].x > 0.5) && (cloud.points[i].x < 5.0))
    {
      if((cloud.points[i].y > -3.0) && (cloud.points[i].y < 3.0))
      {
        x = x + cloud.points[i].x ;
        y = y + cloud.points[i].y ;
        count++ ;
      }
    }
  }
  pos.laser_x = x/count*100;
  pos.laser_y = y/count*100;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "constant_distance_fly");
  ros::NodeHandle n;
  ros::Subscriber cloud_sub = n.subscribe("/pointcloud", 1, cloudCallback);
  ros::Publisher pub = n.advertise<laserscan::Laser>("/laser_send", 1);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
