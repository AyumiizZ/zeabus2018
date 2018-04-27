// please :set nu tabstop=4

// standard include
#include	<iostream>

// include ros system
#include	<ros/ros.h>

// include other file
#include	"calculate_velocity.cpp" // will use pid of this file to calculate force out
#include	"manage_file.cpp" // this file about save / load value of dynamic reconfigure

#include 	<math.h> // for math

// include message
#include 	<nav_msgs/Odometry.h> // type message receive current_state , current_velocity
#include	<geometry_msgs/Twist.h> // type messsage receive target_velocity , out force
#include	<std_msgs/Float64.h>

// include head of service
#include	<zeabus_controller/fix_abs_xy.h>
#include	<zeabus_controller/fix_rel_xy.h>
#include 	<zeabus_controller/fix_abs_depth.h>
#include	<zeabus_controller/ok_position.h>
#include	<zeabus_controller/fix_abs_yaw.h>

// assign the constant value
#define PI 3.14159265
#define epsilon 1.0e-7 // this define I think it mean error by double type when value is 0
// **-------------------------------------------------------------------------------**
static	std::string tune_file = "offset_ku.yaml"; // this for save and load tune file
// **-------------------------------------------------------------------------------**

double*	pid_force = new double[6]; // force output part 01 have calculate by pid
double* offset_force = new double[6]; // force output part 02 have offset {tuning}
double*	sum_force = new double[6]; // sum force of 2 part

// for tuning pid calculate
double 	Kp_position[6] = {0 ,0 ,0 ,0 ,0 ,0}
double	Ki_position[6] = {0 ,0 ,0 ,0 ,0 ,0}
double	Kd_position[6] = {0 ,0 ,0 ,0 ,0 ,0}

// for find error [ x , y , z , roll , pitch , yaw]
double*	current_velocity = new double[6];
double*	target_velocity = new double[6]; // this part will use check want to fix position or not?
double*	current_position = new double[6];
double* target_position = new double[6];

bool can_fix[6] = {true , true , true , true , true , true}; // this tell we have sensor or not?
bool want_fix[6] = {false , false , false , false , false , false}; //  want to go fix_position?
bool already_position[6] = {false , false , false , false , false, false}; // Ok this position?

// this part use to think about should reset target and save new state to target_position
ros::Time last_target_velocity = 0;
ros::Time current_time = 0;
double diff_time = 1; 

bool start_run = true; // this tell to save target state in first time
bool first_time_tune = true; // this use load constant of tune value
bool change_tune = false; // this use check when to save tune value

// function for Subscribe ros
void listen_current_state( const nav_msgs::Odometry message);
void listen_target_velocity( const geometry_msgs::Twist message);

// function for Dynamic Reconfig
void config_constant_PID(zeabus_controller::OffSetConstantConfig &config, uint32_t level);

// function for service
bool service_target_distance(
		zeabus_controller::fix_rel_xy::Request &request , 
		zeabus_controller::fix_rel_xy::Response &response
	); // for get want distance assign with rotation of robot
bool service_target_yaw(
		zeabus_controller::fix_abs_yaw::Request &request ,
		zeabus_controller::fix_abs_yaw::Response &response
	); // for get want yaw
bool service_target_depth(
		zeabus_controller::fix_abs_depth::Request &request ,
		zeabus_controller::fix_abs_depth::Response &response
	); // for get want depth
bool service_ok_position(
		zeabus_controller::ok_position::Request &request ,
		zeabus_controller::ok_position::Response &response
	); // for get ok position

// function subscribe
	
