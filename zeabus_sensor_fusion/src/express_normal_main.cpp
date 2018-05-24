// include 2 part of file

#include	"express_normal_header.h"
#include	"express_normal_detail.h"

int main( int argc , char** argv){
	ros::init( argc , argv , "sensor_fusion");

	ros::NodeHandle nh;

	ros::Subscriber sub_baro_odom = nh.subscribe( "/baro/odom" , 1000 , &listen_baro_odom);
	ros::Subscriber sub_dvl_data = nh.subscribe( "/dvl/data" , 1000 , &listen_dvl_data);
	ros::Subscriber sub_imu_data = nh.subscribe( "/gx4_45_imu/data" , 1000 , &listen_imu_data);

	#ifdef test_01
		std::cout << "finish declare message";
	#endif

	ros::Rate rate(70);
	
	ros::Time current_time, previous_time;

	double world_x = 0 , world_y = 0;
	double previous_velocity_x = 0 , previous_velocity_y = 0;

	ros::Publisher tell_state = nh.advertise<nav_msgs::Odometry>("/auv/state", 1000);

	while( nh.ok()){
		current_time = ros::Time::now();
		if( get_dvl && get_imu && get_pressure){
			diff_time = (current_time - previous_time).toSec();
			double robot_x = ( current_velocity_x + previous_velocity_x 	)
							/ 2 * diff_time * cos( pitch );  
			double robot_y = ( current_velocity_y + previous_velocity_y )
							/ 2 * diff_time * cos( roll );
			double adding_x = robot_x * cos( yaw ) + robot_y * cos( yaw + PI/2);
			double adding_y = robot_x * sin( yaw ) + robot_y * sin( yaw + PI/2);
            if( absolute_check(adding_x)) auv_state.pose.pose.position.x += adding_x;
			if( absolute_check(adding_y)) auv_state.pose.pose.position.y += adding_y;
			#ifdef data_02
				ROS_INFO("----------------------------print data-------------------------------\n");
				ROS_INFO("diff_time\t:\t%.2lf" , diff_time);
				ROS_INFO("adding_robot\t:\t%.4lf\t%.4lf"
							, robot_x , robot_y);	 
				ROS_INFO("adding_world\t:\t%.4lf\t%.4lf"
							, adding_x , adding_y);	 
				ROS_INFO("current_pos\t:\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf"
							, auv_state.pose.pose.position.x , auv_state.pose.pose.position.y 
							, auv_state.pose.pose.position.z , roll , pitch , yaw);
			#endif		
			previous_velocity_x = auv_state.twist.twist.linear.x;
			previous_velocity_y = auv_state.twist.twist.linear.y;
			tell_state.publish( auv_state );
		}
		else{
			ROS_FATAL("DON'T ALREADY");
		}
		rate.sleep();
		ros::spinOnce();
		previous_time = current_time;
	}
}
