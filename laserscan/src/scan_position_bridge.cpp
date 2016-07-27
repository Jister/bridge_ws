#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>

struct target{
      float x;
      float y;
      int n;
};

geometry_msgs::PoseStamped pos;
geometry_msgs::PoseStamped pos_pre;
target a[20];
target b[20];

float ref_x;
float ref_y;
bool set_ref;
int ref_num;
int ref_num_pre;

void cloudCallback (const sensor_msgs::PointCloud cloud)
{
      int count = 0;
      float x_n = cloud.points[0].x ;
      float y_n = cloud.points[0].y ;
      int cnt=0;

      memset(a , 0 , sizeof(a));
      memset(b , 0 , sizeof(b));
      for(int i=0;i<cloud.points.size();i++)      
      {
            if(sqrt( (cloud.points[i].x - x_n) * (cloud.points[i].x - x_n) + (cloud.points[i].y - y_n) * (cloud.points[i].y - y_n) )<1)
            {
                  a[count].x += cloud.points[i].x;
                  a[count].y += cloud.points[i].y;
                  a[count].n += 1 ; 
                  x_n = cloud.points[i].x;
                  y_n = cloud.points[i].y;
             }else
            {
                  count++;
                  x_n = cloud.points[i].x;
                  y_n = cloud.points[i].y;
                  a[count].x += cloud.points[i].x;
                  a[count].y += cloud.points[i].y;
                  a[count].n += 1 ; 
             }
      }

      for(int i = 0; i<20; i++){
            if(a[i].n > 10){
                  b[cnt].x = a[i].x / a[i].n ;
                  b[cnt].y = a[i].y / a[i].n ;
                  b[cnt].n = a[i].n ;
                  ROS_INFO("x%d:%f" , cnt , b[cnt].x);
                  ROS_INFO("y%d:%f" , cnt , b[cnt].y);
                  ROS_INFO("part%d\n" , cnt);
                  cnt++;
            }
      }
      ROS_INFO("one scan \n\n");  
}

int main(int argc, char **argv)
{
      ros::init(argc, argv, "scan_postition");
      ros::NodeHandle n;
      ros::Subscriber cloud_sub = n.subscribe("/pointcloud", 1, cloudCallback);
      ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);
      ros::Rate loop_rate(20);
      
      while(ros::ok())
      {
             ros::Rate check_loop_rate(20);
             while((b[0].x < 0.001) && ros::ok()){
                  ROS_INFO("No reference \n\n") ;
                  ros::spinOnce();
                  check_loop_rate.sleep();
                  set_ref = false;
             }

             ref_num = 0 ;
             for(int i = 0 ; i<20 ; i++){
                   if( b[i].n > 0 ){
                          ref_num = ref_num + 1 ;
                   }
             }
             ROS_INFO("ref_num %d\n\n",ref_num) ;
             ROS_INFO("ref_num_pre %d\n\n",ref_num_pre) ;

             if(ref_num_pre != ref_num){
                  set_ref =false ; 
             }

             if(!set_ref){
                   if(ref_num == 1){
                          ref_x = b[0].x ; 
                          ref_y = b[0].y ; 
                   }
                   if(ref_num == 2){
                          ref_x = ( b[0].x + b[1].x ) / 2; 
                          ref_y = ( b[0].y + b[1].y ) / 2; 
                   }
                   set_ref = true;
                   ROS_INFO("Reset reference \n\n") ;
             }

             if(ref_num == 1){
                   if(ref_num == ref_num_pre){
                          pos.pose.position.x = ref_x - b[0].x ;
                          pos.pose.position.y = b[0].y - ref_y ;
                   }else{
                          pos.pose.position.x = ref_x - b[0].x + pos_pre.pose.position.x ;
                          pos.pose.position.y = b[0].y - ref_y + pos_pre.pose.position.y ;
                   }  
             }
             if(ref_num == 2){
                   if(ref_num == ref_num_pre){
                          pos.pose.position.x = ref_x - (b[0].x + b[1].x)/2 ;
                          pos.pose.position.y = (b[0].y + b[1].y)/2 - ref_y ;
                   }else{
                          pos.pose.position.x = ref_x - (b[0].x + b[1].x)/2 + pos_pre.pose.position.x ;
                          pos.pose.position.y = (b[0].y + b[1].y)/2 - ref_y + pos_pre.pose.position.y ;
                   }  
             }
             pos.header.stamp = ros::Time::now();
             pos_pre = pos;
             ref_num_pre = ref_num;
             pub.publish(pos);
             ros::spinOnce();
             loop_rate.sleep();
      }
      return 0;
}