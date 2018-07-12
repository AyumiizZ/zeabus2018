/* 
    --------------------- this file will plan to filter pressure sensor ------------------------
		by use if else and use distance you have to ok or get new value when have more distance
*/


#include	<iostream>

#include	<ctime>

#include	<ros/ros.h>
#include	<ros/package.h>

#include	<nav_msgs/Odometry.h>

// --------------------------------- declare for print data -----------------------------------
#define 	general_data
// --------------------------------------------------------------------------------------------

// function for listen value from pressure sensro
void listen_baro_odom( const nav_msgs::Odometry message_odometry);
// function for detect value of pressure
void absolute_detect( double diff , double preblem);

// declare variable for pressure_sensor
double ok_data , new_data;
bool get_new_value;

int main( int argc , char** argv){
	ros::init( argc , argv , "filter_pressure");

	ros::NodeHandle nh;

	// init variable
	std::string input_topic = "/baro/odom";
	int input_size = 5; // queue for subscriber

	std::string output_topic = "/pressure_sensor";
	int output_size = 5; // queue for publisher

	int frequency_time = 100;

	int count = 0;
	int limit_value = 40; // limit value will ok new value
	double ok_different = 0.5; // ok different to use 

	std::time_t offset_time = std::time(NULL); // last time have recieve data from topic
	std::time_t limit_time = 10; // limit value time have to warning not receive data
	std::time_t now_time = std::time(NULL); // now time
	get_new_value = false;

	ros::Rate rate(frequency_time);

	// declare about subscriber and publisher
	ros::Subscriber sub_baro_odom = nh.subscribe( input_topic , input_size , &listen_baro_odom);
	ros::Publisher tell_baro_odom = nh.advertise< nav_msgs::Odometry>
											( output_topic , output_size);

	// declare about rate of while or rate for calculating
	std::cout << "input topic is " << input_topic << "\n";
	std::cout << "input size is " << input_size << "\n";
	std::cout << "output topic is " << output_topic << "\n";
	std::cout << "output size is " << output_size << "\n"; 
	std::cout << "frequency time is " << frequency_time << "\n";
	std::cout << "ok different is " << ok_different << "\n";
	std::coit << "rate time is " << frequency_time << "\n";

	while( nh.ok() ){
		if( get_new_value ){
			if( absolute_detect( ok_different , ok_data - new_data ) ){
				count = 0;
				ok_data = new_data;
			}
			offset_time = std::time(NULL);
			get_new_value = false;
		}
		else{
			now_time = std::time(NULL);
			if( now_time - offset_time > limit_time){
				ROS_FATAL(" Warning receive ok data pressure very slow over limit_time");
			}
		}
		rate.sleep();
		ros::spincOnce();
	}

}

void listen_baro_odom( const nav_msgs::Odometry message_odometry){
	new_data = message_odometry.PoseWithCovariance.pose.position.z;
	get_new_value = true;
}

bool absolute_detect(double diff , double problem){
    if( diff > problem) return false;
    else if( diff < -1*problem) return false;
    else return true;
}
