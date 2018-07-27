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

void fusion_depth_velocity_imu( const nav_msgs::Odometry message_odometry, 
								const geometry_msgs::TwistWithCovarianceStamped message_twist ,
								const sensor_msgs::Imu message_orieantation);

// this part show about current_position of x y z
double world_x=0 , world_y=0 , world_z=0;

// this part have for collect data of previous velocity
double previous_velocity_x = 0 , previous_velocity_y = 0;
double add_world_x , add_world_y;
double add_robot_x , add_robot_y;

// this part have for collect previous depth for find velocity 
double previous_depth = 0 ;

bool start_run = true;

ros::Publisher tell_auv_state;

ros::Time previous_time , current_time;

nav_msgs::Odometry msgs_auv_state;
