#include	<iostream> // include standard cpp
#include	<math.h> // include math for calculate

#include	<boost/bind.hpp>

#define 	PI 3.14159265

#include	<ros/ros.h> // include standard ros

// include message_filter for make sensor fusion
#include	<message_filters/subscriber.h>
#include	<message_filters/time_synchronizer.h>
#include	<message_filters/sync_policies/exact_time.h>
#include	<message_filters/sync_policies/approximate_time.h>

// include type of any message
#include	<nav_msgs/Odometry.h>
#include	<geometry_msgs/TwistWithCovarianceStamped.h>
#include	<sensor_msgs/Imu.h>

// include part for convert quaternion to roll  pitch yaw
#include	<tf/transform_datatypes.h>
#include	<tf/transform_listener.h>

// include for try to use boost
//#include	<boost/thread.hpp>

// --------------------------- declare for print data ---------------------------------------
#define test_01
#define data_01
#define data_02
// ------------------------------------------------------------------------------------------

void listen_baro_odom( const nav_msgs::Odometry message_odometry);
void listen_dvl_data( const geometry_msgs::TwistWithCovarianceStamped message_twist);
void listen_ime_data( const sensor_msgs::Imu message_orieantation);

// declare velocity form dvl data
double new_velocity_x , new_velocity_y;

// declare for get orientation form imu
double roll , pitch , yaw;

// declare for check 3 subscribe
bool get_dvl = false, get_imu = false, get_pressure = false;

bool start_run = true;

