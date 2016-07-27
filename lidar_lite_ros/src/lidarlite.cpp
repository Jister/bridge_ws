#include "ros/ros.h"
#include "lidar_lite.h"
#include "lidar_lite_ros/Lidarlite.h"
#include "std_msgs/String.h"
#include <time.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarlite_node");
  ros::NodeHandle n;
  ros::Publisher lidar_pub = n.advertise<lidar_lite_ros::Lidarlite>("lidar_distance", 1);
  ros::Rate loop_rate(20);

  int fd;
  int value;
  unsigned char status;
  lidar_lite_ros::Lidarlite msg;

  fd = lidar_init(false);
  if (fd == -1) {
    printf("initialization error\n");
  }
  while (ros::ok())
  {
    value = lidar_read(fd);
    status = lidar_status(fd);
    ROS_INFO("%3.0d cm", value);
    msg.distance = value;
    lidar_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
