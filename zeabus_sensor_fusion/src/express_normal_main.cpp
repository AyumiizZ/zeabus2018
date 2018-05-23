// include 2 part of file

#include	"normal_fusion_header.h"
#include	"normal_fusion_detail.h"

int main( int argc , char** argv){
	ros::init( argc , argv , "sensor_fusion");

	ros::NodeHandle nh;

	ros::Subscriber sub_baro_odom = nh.subscriber( "/baro/odom" , 1000 , &listen_baro_odom);
	ros::Subscriber sub_dvl_data = nh.subscriber( "/dvl/data" , 1000 , &listen_dvl_data);
	ros::Subscriber sub_baro_odom = nh.subscriber( "/gx4_45_imu/data" , 1000 , &listen_baro_odom);

	#ifdef test_01
		std::cout << "finish declare message";
	#endif

	ros::Rate rate(100);
	
	ros::Time current_time, previous_time;

	nav_msgs::Odometry auv_message;

	double world_x = 0 , world_y = 0;
	double current_velocity_x = 0 ; current_velocity_y = 0 ;

	while( nh.ok()){
		current_time = ros::Time::now();
		if( start_run ){
			world_x = 0 ; 
			world_y = 0 ;
		}
		else if( get_dvl && get_imu && get_pressure){
			double diff_time = (current_time - previous_time).
		}
		else{
		
		}
		rate.sleep();
		ros::spinOnce();
		previous_time = current_time;
	}
}
