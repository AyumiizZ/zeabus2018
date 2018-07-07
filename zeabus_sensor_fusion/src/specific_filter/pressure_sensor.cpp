/* --------------------- this file will plan to filter pressure sensor ------------------------
		by use if else and use distance you have to ok or get new value when have more distance
*/


#include	<iostream>

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
double previous_data , ok_data , new_data;

int main( int argc , char** argv){
	ros::init( argc , argv , "filter_pressure");

	ros::NodeHandle nh;

	// init variable
	std::string input_topic = "/baro/odom";
	int input_size = 5;
	std::string output_topic = "/pressure_sensor";
	int output_size = 5;
	double frequency_time = 100;
	double limit_value = 40;
	double ok_different = 0.5;

	// declare about subscriber and publisher
	ros::Subscriber sub_baro_odom = nh.subscribe( input_topic , input_size , &listen_baro_odom);
	ros::Publisher tell_baro_odom = nh.advertise< nav_msgs::Odometry>
											( output_topic , output_size);

	std::cout << "input topic is " << input_topic << "\n";
	std::cout << "input size is " << input_size << "\n";
	std::cout << "output topic is " << output_topic << "\n";
	std::cout << "output size is " << output_size << "\n"; 
	std::cout << "frequency time is " << frequency_time << "\n";
	std::cout << "ok different is " << ok_different << "\n";
	
	while( nh.ok() ){
		
	}
}

bool absolute_detect(double diff , double problem){
    if( diff > problem) return false;
    else if( diff < -1*problem) return false;
    else return true;
}
