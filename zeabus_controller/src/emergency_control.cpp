#include	<iostream>
#include 	<ros/ros.h>
#include 	<cstdlib>
#include 	<std_msgs/Bool.h>
#include	<nav_msgs/Odometry.h>

#include	"manage_control.cpp"

void listen_current_state( const nav_msgs::Odometry message);
bool active_close = false;

int main(int argc, char **argv){
	ros::init(argc , argv , "manage_controller");
	ros::NodeHandle nh;
	ros::Subscriber sub_state = nh.subscribe("/auv/state" , 1000, &listen_current_state);
	ros::spin();
}

void listen_current_state( const nav_msgs::Odometry message){
	double depth = message.pose.pose.position.z;
	if( active_close && ( depth > -1 ) ){
		manage_control.kill_node("Controller");
		manage_control.kill_node("pure_thruster_mapper");
		manage_control.kill_node("sensor_fusion");
		std::cout << "I close that\n";	
		active_close = false;
	}
	else if( depth < -1.5 ){
		active_close = true;
		std::cout << "Now active close\n";
	}
}
