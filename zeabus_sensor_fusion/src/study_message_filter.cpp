#include	<iostream> // include standard cpp

#include	<ros/ros.h> // include ros system

// include message_filter for make sensor fusion
#include	<message_filters/subscriber.h>
#include 	<message_filters/time_synchronizer.h>

// include type of any message
#include	<nav_msgs/Odometry.h>
#include	<geometry_msgs/Twist.h>
#include	<sensor_msgs/Imu.h>

void test_sensor_fusion( const nav_msgs::Odometry message_odometry , 
						 const geometry_msgs::Twist message_twist , 
						 const sensor_msgs::Imu message_orieantation){
	std::cout << "Test in function call back \n"; 
}

int main(int argc , char** argv){
	ros::init( argc , argv , "sensor_fusion");

	ros::NodeHandle nh;

	message_filters::Subscriber< nav_msgs::Odometry> depth_sub(nh , "/baro/odom" , 1);
	
	message_filters::Subscriber< geometry_msgs::Twist> velocity_xy_sub(nh , "/dvl/data" , 1);

	message_filters::Subscriber< sensor_msgs::Imu> orieantation_sub(nh , "/gx4_45_imu/data" , 1);
	
	// The synchronize in c++ can be up to 9 channels
	message_filters::TimeSynchronizer< nav_msgs::Odometry , geometry_msgs::Twist , 
										sensor_msgs::Imu > sync( depth_sub , velocity_xy_sub 
																, orieantation_sub , 10);
	
	sync.registerCallback( boost::bind(&test_sensor_fusion, _1, _2 , _3));

	ros::spin();

	return 0;
}
