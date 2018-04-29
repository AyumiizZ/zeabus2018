// please :set nu tabstop=4

// ------------------------------------define for debug code -------------------------------- //
#define	test_01 // test state
#define test_02 // print test
#define print_data // print data
// ------------------------------------------------------------------------------------------ //

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
#include	<geometry_msgs/Point message>
#include	<std_msgs/Bool.h>
#include	<std_msgs/Float64.h>

// include head of service
#include	<zeabus_controller/fix_abs_xy.h>
#include	<zeabus_controller/fix_rel_xy.h>
#include 	<zeabus_controller/fix_abs_depth.h>
#include	<zeabus_controller/ok_position.h>
#include	<zeabus_controller/fix_abs_yaw.h>

// include part for convert quaternion to roll pitch yaw
#include 	<tf/transform_datatypes.h>
#include	<tf/transform_listener.h>

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
double 	K_velocity[6] =  {0 ,0 ,0 ,0 ,0 ,0}
double 	Kp_velocity[6] = {0 ,0 ,0 ,0 ,0 ,0}
double	Ki_velocity[6] = {0 ,0 ,0 ,0 ,0 ,0}
double	Kd_position[6] = {0 ,0 ,0 ,0 ,0 ,0}

// for these variable [ x , y , z , roll , pitch , yaw]
double	bound_force[6] = { 4, 4, 8, 2, 2, 2};
double*	current_velocity = new double[6];
double*	target_velocity = new double[6]; // this part will use check want to fix position or not?
double*	current_position = new double[6];
double* target_position = new double[6];
double* world_error = new double[6]; // this part will calculate error from sensor
double* robot_error = new double[6]; // this part will use to calculate force and calculate form
									 // world_error
double ok_error = { 0.05 , 0.05 , 0.05 , 0.1 , 0.1 , 0.1}; // for calculate error you ok

bool can_fix[6] = {true , true , true , true , true , true}; // this tell we have sensor or not?
bool want_fix[6] = {false , false , false , false , false , false}; //  want to go fix_position?
bool already_position[6] = {false , false , false , false , false, false}; // Ok this position?

// this part use to think about should reset target and save new state to target_position
ros::Time last_target_velocity = 0;
ros::Time current_time = 0;
double diff_time = 1; // this variable 

bool start_run = true; // this tell to save target state in first time
bool reset_position = true;
bool first_time_tune = true; // this use load constant of tune value
bool change_tune = false; // this use check when to save tune value

// function for Subscribe ros
void listen_current_state( const nav_msgs::Odometry message);
void listen_target_velocity( const geometry_msgs::Twist message);

// function for Dynamic Reconfig and function about dynamic reconfig
void config_constant_PID(zeabus_controller::OffSetConstantConfig &config, uint32_t level);
void reset_all_I();
void set_all_tunning();
void reset_specific_position( int number);
void reset_specific_velocity( int number);

// about how to tuning
int mode_control = 1; // mode 1 , 2 is depth about offset and PID 3 ,4 roll pitch 
					  // 5 load PID in normal situation 6 common

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
bool service_change_mode(
		zeabus_controller::change_mode::Request &request , 
		zeabus_controller::change_mode::Response &response
	); // for get mode

// function subscribe for testing
#ifdef test_01
	void test_current_state(const geometry_msgs::Point message);
	void test_current_orientation(const zeabus_controller::orientation message);
#endif

// declare variable from code
find_velocity::second_case *PID_position; // use to calculate force
find_velocity::second_case *PID_velocity; // use to calculate force when calculate about r p y
manage_PID_file PID_file(tune_file); // use to save or download

double convert_range_radian( double problem);// convert [ -PI , PI] to [0 , 2PI]
double bound_value_radian( double problem);// bound value to in [0 , 2PI]

double absolute( double problem);
