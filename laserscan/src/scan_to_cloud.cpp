#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"

laser_geometry::LaserProjection projector;
sensor_msgs::PointCloud cloud;

void scanCallback (const sensor_msgs::LaserScan scan)
{
  projector.projectLaser(scan, cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan", 1, scanCallback);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud", 1);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    pub.publish(cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
