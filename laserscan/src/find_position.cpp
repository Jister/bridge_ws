#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <math.h>
#include <vector>

using namespace std;

struct Point
{
	double x;
	double y;
};

double Length(Point p1, Point p2)
{
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

struct Line
{
	Point start;
	Point end;
	Point mean;
	double k;
	double R;
};

class GetPosition
{
public: 
	GetPosition();
private:
	ros::NodeHandle n;
	ros::Subscriber scan_sub;
	ros::Publisher pub;
	ros::Publisher marker_pub;
	
	vector<Line> lines;

	void scanCallback(const sensor_msgs::LaserScan laser);
	void segment(vector<Point> input);
};

GetPosition::GetPosition()
{
	scan_sub = n.subscribe("/scan", 1, &GetPosition::scanCallback, this);
	pub = n.advertise<geometry_msgs::PointStamped>("/pier_position", 5);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}


void GetPosition::segment(vector<Point> input)
{
	// if(input.size() > 2)
	// {
	// 	Point point_1 = input[0];
	// 	Point point_2 = input[input.size()-1];
	// 	double A = point_1.y - point_2.y;
	// 	double B = point_2.x - point_1.x;
	// 	double C = point_1.x * point_2.y - point_2.x * point_1.y;
	// 	double D = sqrt(A * A + B * B);
	// 	double max_d = 0;
	// 	int max_pos = 0;
	// 	for(int i = 0; i< input.size(); i++)
	// 	{
	// 		//(y1 - y2) * x + (x2 - x1) * y + (x1 * y2 - x2 * y1) = 0
	// 		//d = |[(y1 - y2) * x + (x2 - x1) * y + (x1 * y2 - x2 * y1)]/sqrt((y1 - y2)^2 + (x2 - x1)^2)|
	// 		double distance = fabs((A * input[i].x + B * input[i].y + C)/D);
	// 		if(distance > max_d)
	// 		{
	// 			max_pos = i;
	// 			max_d = distance;
	// 		}
	// 	}
	// 	if(max_d < 0.1)
	// 	{
	// 		Line l;
	// 		l.start = input[0];
	// 		l.end = input[input.size()-1];
	// 		l.k = 0;
	// 		l.R = 0;

	// 		lines.push_back(l);
	// 		return;
	// 	}else
	// 	{
	// 		vector<Point> v1, v2;
	// 		for(int i = 0; i < max_pos; i++)
	// 		{
	// 			v1.push_back(input[i]);
	// 		}
	// 		for(int i = max_pos+1; i < input.size(); i++)
	// 		{
	// 			v2.push_back(input[i]);
	// 		}
	// 		segment(v1);
	// 		segment(v2);
	// 	}
	// }else{
	// 	return;
	// }		

	//Method of least squares
	int size = input.size();
	if(size > 2)
	{
		double xmean = 0;
		double ymean = 0;
		double x2mean = 0;
		double y2mean = 0;
		double xymean = 0;
		for(int i = 0; i < size; i++)
		{
			xmean += input[i].x;
			ymean += input[i].y;
			x2mean += input[i].x * input[i].x;
			y2mean += input[i].y * input[i].y;
			xymean += input[i].x * input[i].y;
		}
		xmean /= size;
		ymean /= size;
		x2mean /= size;
		y2mean /= size;
		xymean /= size;

		double k,b,tmp,r;
		if(tmp = x2mean - xmean * xmean)
		{
			k = (xymean - xmean * ymean) / tmp;
			b =  ymean - k * xmean;
		}else
		{
			k = 1;
			b = 0;
		}
		r = (xymean - xmean * ymean)/sqrt((x2mean - xmean * xmean) * (y2mean - ymean * ymean));

		double A = k;
		double B = -1;
		double C = b;
		double D = sqrt(A * A + B * B);
		double max_d = 0;
		int max_pos = 0;
		for(int i = 0; i< input.size(); i++)
		{
			//(y1 - y2) * x + (x2 - x1) * y + (x1 * y2 - x2 * y1) = 0
			//d = |[(y1 - y2) * x + (x2 - x1) * y + (x1 * y2 - x2 * y1)]/sqrt((y1 - y2)^2 + (x2 - x1)^2)|
			double distance = fabs((A * input[i].x + B * input[i].y + C)/D);
			if(distance > max_d)
			{
				max_pos = i;
				max_d = distance;
			}
		}
		if(max_d < 0.1)
		{
			Line l;
			l.start = input[0];
			l.end = input[input.size()-1];
			l.mean.x = xmean;
			l.mean.y = ymean;
			l.k = A;
			l.R = r;

			lines.push_back(l);
			return;
		}else
		{
			vector<Point> v1, v2;
			for(int i = 0; i < max_pos; i++)
			{
				v1.push_back(input[i]);
			}
			for(int i = max_pos+1; i < input.size(); i++)
			{
				v2.push_back(input[i]);
			}
			segment(v1);
			segment(v2);
		}
	}else{
		return;
	}	
}


void GetPosition::scanCallback(const sensor_msgs::LaserScan laser)
{
	vector<Point> cloud;
	for(int i=0; i<laser.ranges.size(); i++)
	{
		if(laser.ranges[i] > 1.0 && laser.ranges[i] < laser.range_max)
		{
			double angle = laser.angle_min + i * laser.angle_increment;
			double x = laser.ranges[i] * cos(angle);
			double y = laser.ranges[i] * sin(angle);
			Point point;
			point.x = x;
			point.y = y;
			cloud.push_back(point);
		}
			
	}

	vector<Point> temp;
	for(int i = 0; i < cloud.size(); i++)
	{
		if(temp.empty())
		{
			temp.push_back(cloud[i]);
		}else
		{
			if(Length(cloud[i], cloud[i-1]) < 0.1)
			{
				temp.push_back(cloud[i]);
			}else
			{
				segment(temp);
				temp.clear();
				temp.push_back(cloud[i]);
			}
		}
	}
	if(!temp.empty())
	{
		segment(temp);
	}

	double min_dist = 0;
	int min_dis_pos = 0;
	bool flag = false;
	
	for(int i = 0; i < lines.size(); i++)
	{
		if(Length(lines[i].start, lines[i].end)>1.5 && Length(lines[i].start, lines[i].end)<2.7)
		{
			flag = true;
			double dist = sqrt(lines[i].mean.x*lines[i].mean.x + lines[i].mean.y*lines[i].mean.y);
			if(min_dis_pos == 0)
			{
				min_dist = dist;
				min_dis_pos = i;
			}else
			{

				if(dist < min_dist)
				{
					min_dist = dist;
					min_dis_pos = i;
				}
			}
		}
	
	}
	geometry_msgs::Point p1, p2; 
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/laser"; 
	line_list.header.stamp = ros::Time::now(); 
	line_list.ns = "find_position";  
	line_list.action = visualization_msgs::Marker::ADD; 
	line_list.type = visualization_msgs::Marker::LINE_LIST;  
	line_list.scale.x = 0.1;  
	line_list.color.r = 1.0;  
	line_list.color.a = 1.0;  

	if(flag)
	{
		p1.x = lines[min_dis_pos].start.x;
		p1.y = lines[min_dis_pos].start.y;
		p1.z = 0;
		line_list.points.push_back(p1);
		p2.x = lines[min_dis_pos].end.x;
		p2.y = lines[min_dis_pos].end.y;
		p2.z = 0;
		line_list.points.push_back(p2);

		ROS_INFO("length:%f",Length(lines[min_dis_pos].start, lines[min_dis_pos].end));
		ROS_INFO("k:%f",lines[min_dis_pos].k);
		ROS_INFO("r:%f",lines[min_dis_pos].R);
		ROS_INFO("x:%f",lines[min_dis_pos].mean.x);
		ROS_INFO("y:%f",lines[min_dis_pos].mean.y);

		geometry_msgs::PointStamped pos;
		pos.header.frame_id = "/laser";
		pos.header.stamp = ros::Time::now();
		pos.point.x = lines[min_dis_pos].mean.x;
		pos.point.y = lines[min_dis_pos].mean.y;
		pos.point.z = 0;

		pub.publish(pos);
		marker_pub.publish(line_list);
	}
	
	lines.clear();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_position");
	GetPosition Get_position;
	ros::spin();
}