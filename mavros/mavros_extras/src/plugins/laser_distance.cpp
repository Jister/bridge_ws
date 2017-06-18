#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/LaserDistance.h>
#include <ros/console.h>

namespace mavplugin{

class LaserDistancePlugin : public MavRosPlugin{

public:
	 LaserDistancePlugin():
	 laser_distance_nh("~laser_distance"),
	 uas(nullptr)
    { };

    void initialize(UAS &uas_){
    	uas = &uas_;
        
    	laser_distance_nh.param<std::string>("frame_id", frame_id, "laser_distance");
        //subcribe the topic and excute the callback function
    	laser_distance_sub = laser_distance_nh.subscribe("/laser_send",2,&LaserDistancePlugin::laser_distance_send_cb,this);
        pier_sub = laser_distance_nh.subscribe("/pier_position",2,&LaserDistancePlugin::pier_position_cb,this);

    }
    
    std::string get_name() {
		return "laser_distance";
	}


     const message_map get_rx_handlers() {
		return {
			     
		};
	}


private:
	ros::NodeHandle laser_distance_nh;
	ros::Subscriber laser_distance_sub;
    ros::Subscriber pier_sub;
	UAS *uas;

	std::string frame_id;

    mavros_extras::LaserDistance message;
    time pos_stamp;

    void laser_distance_send(float a, float b, float c, float d){
    	mavlink_message_t laser_distance_msg;

    	mavlink_msg_laser_distance_pack_chan(UAS_PACK_CHAN(uas),&laser_distance_msg,a,b,c,d); //pack
    	UAS_FCU(uas)->send_message(&laser_distance_msg); //send
        
       // ROS_INFO("float_b %d %d", laser_distance_msg.seq,laser_distance_msg.len);
    	
    }
    
    //callbacks
    void laser_distance_send_cb(const mavros_extras::LaserDistance &msg){
        stamp t = ros::Time::now();
        if((t.sec - pos_stamp.sec) >= 1)
        {
            laser_distance_send(msg.min_distance,msg.angle,0,0);
        }else
        {
            laser_distance_send(msg.min_distance,msg.angle,message.laser_x,message.laser_y);
        }       
    }
    void pier_position_cb(const geometry_msgs::PointStamped &msg){
        pos_stamp = msg.header.stamp;
        message.laser_x = msg.point.x;
        message.laser_y = -msg.point.y;
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::LaserDistancePlugin, mavplugin::MavRosPlugin)
