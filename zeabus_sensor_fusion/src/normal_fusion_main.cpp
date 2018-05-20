// include 2 part of file

#include	"normal_fusion_header.h"
#include	"normal_fusion_detail.h"

int main( int argc , char** argv){
	ros::init( argc , argv , "sensor_fusion");

	ros::NodeHandle nh;

	#ifdef test_01
		std::cout << "finish set node\n";
	#endif

// declare message_filter
	message_filters::Subscriber< nav_msgs::Odometry > depth_sub
								( nh , "/baro/odom" , 1 );

	message_filters::Subscriber< geometry_msgs::TwistWithCovarianceStamped > velocity_xy_sub
								( nh , "/dvl/data" , 1);

	message_filters::Subscriber< sensor_msgs::Imu > orieantation_sub
								( nh , "/gx4_45_imu/data" , 1);

	#ifdef test_01
		std::cout << "finish declare message_filters\n";
	#endif

// The synchronize in c++ can be up to 9 channels
	message_filters::TimeSynchronizer< 	nav_msgs::Odometry
										, geometry_msgs::TwistWithCovarianceStamped
										, sensor_msgs::Imu > sync ( depth_sub , velocity_xy_sub
																	, orieantation_sub , 10);

	sync.registerCallback( fusion_depth_velocity_imu);

	#ifdef test_01
		std::cout << "finish register callback\n";
	#endif

	tell_auv_state = nh.advertise< nav_msgs::Odometry >( "/auv/state" , 1000);

	#ifdef test_01
		std::cout << "finish declare publisher\n";
	#endif



	ros::spin();

	return 0;
}
