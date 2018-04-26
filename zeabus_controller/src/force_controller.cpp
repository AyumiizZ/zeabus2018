#include	<ros/ros.h> // Use ros system
#include	<ros/time.h> // Use time by ros system

#include 	<iostream> // Base of CPP Language
#include 	<stdlib.h> 
#include 	<vector>
#include	<cmath>
#include	<string>
#include	<Vector3>
#include	<queue> 

#include 	"manage_file.cpp" // Use Load or Save dynamic value
#include 	"calculate_force.cpp" // Use calculate force form acceleration

// 2 line will use between quaternion with roll pitch yaw
#include 	<tf/transform_datatypes.h>
#include 	<tf/transform_listener.h>

#include	<nav_msgs/Odometry.h> // Include message for receive auv_state
#include 	<geometry_msgs/Twist.h> // Include message for send to thruster_mapper
#include 	<geometry_msgs/Point.h> 
#include 	<geometry_msgs/Pose.h> 
#include 	<std_msgs/Float64.h> // Include message for receive or send variable type float 64
#include 	<std_msgs/Float32.h> // Include message for receive or send variable type float 32
#include 	<std_msgs/Int16.h> // Include message for receive type int
#include 	<std_msgs/String.h> // Include message for receive type string
#include 	<std_msgs/Bool.h> // Include message for receive type bool
#include	<math.h>
#include	<nav_msgs/Odometry.h>
#include	<sensor_msgs/Imu.h>
#include	<zeabus_controller/point_xy.h>
#include	<zeabus_controller/orientation.h>
#include	<modbus_ascii_ros/Switch.h>
#include	<dynamic_reconfigure/server.h>
#include	<zeabus_controller/PIDConstantConfig.h>

//#include	<zeabus_controller/drive_x.h> //unused in code
#include	<zeabus_controller/message_service.h>
#include	<zeabus_controller/fix_abs_xy.h>
#include	<zeabus_controller/fix_abs_x.h>
#include	<zeabus_controller/fix_abs_y.h>
#include	<zeabus_controller/fix_abs_depth.h>
#include	<zeabus_controller/fix_abs_yaw.h>
#include	<zeabus_controller/fix_rel_xy.h>
#include	<zeabus_controller/ok_position.h>

//void listen_mode_control(const std_msgs::Int16 message);
//for testing
void test_current_state(const geometry_msgs::Point message);
void test_current_orientation(const zeabus_controller::orientation message);
//setup function of ros
void listen_current_state(const nav_msgs::Odometry message);
void listen_target_velocity(const geometry_msgs::Twist message);
void listen_target_position(const geometry_msgs::Point message);
void listen_target_depth(const std_msgs::Float64 message);
void listen_absolute_yaw(const std_msgs::Float64 message);
void listen_real_yaw(const std_msgs::Float64 message);
void listen_absolute_xy(const zeabus_controller::point_xy message);
void listen_absolute_orientation(const zeabus_controller::orientation message);
//setup function of service
bool service_target_xy(zeabus_controller::fix_rel_xy::Request &request, zeabus_controller::fix_rel_xy::Response &response);
bool service_target_depth(zeabus_controller::fix_abs_depth::Request &request, zeabus_controller::fix_abs_depth::Response &response);
bool service_target_yaw(zeabus_controller::fix_abs_yaw::Request &request, zeabus_controller::fix_abs_yaw::Response &response);
bool service_target_x(zeabus_controller::fix_abs_x::Request &request, zeabus_controller::fix_abs_x::Response &response);
bool service_target_y(zeabus_controller::fix_abs_y::Request &request, zeabus_controller::fix_abs_y::Response &response);

int main(int argc, char **argv){
//setup ros system(Initialization)
	ros::init(argc, argv, "force_controller")//Initializing the roscpp Node
	ros::NodeHandle nh;//Starting the roscpp Node
	//ros::Subscriber sub_mode = nh.subscribe("/mode_control", 1000, &listen_mode_control);
//test topic
	ros::Subscriber test_state = nh.subscribe("/test/point" , 1000, &test_current_state);//(topic, number of max input, function's address)
	ros::Subscriber test_orientation = nh.subscribe("/test/orientation", 1000, &test_current_orientation);
//Sub topic
	ros::Subscriber sub_state = nh.subscribe("/auv/state" , 1000, &listen_current_state);
	ros::Subscriber sub_target_velocity = nh.subscribe("/zeabus/cmd_vel", 1000, &listen_target_velocity);
	ros::Subscriber sub_target_position = nh.subscribe("/cmd_fix_position", 1000, &listen_target_position);
	ros::Subscriber sub_target_depth = nh.subscribe("/fix/abs/depth", 1000, &listen_target_depth);
	ros::Subscriber sub_absolute_yaw = nh.subscribe("/fix/abs/yaw", 1000, &listen_absolute_yaw);
	ros::Subscriber sub_real_yaw = nh.subscribe("/fix/rel/yaw", 1000, &listen_real_yaw);
	ros::Subscriber sub_absolute_xy = nh.subscribe("fix/abs/xy", 1000, &listen_absolute_xy);
	ros::Subscriber sub_absolute_orientation = nh.subscribe("/fix/abs/orientation", 1000, &listen_absolute_orientation);	
//service topic
	ros::ServiceServer ser_cli_target_xy = nh.advertiseService("/fix_abs_xy", service_target_xy);
        ros::ServiceServer ser_cli_target_distance = nh.advertiseService("/fix_rel_xy", service_target_distance);
        ros::ServiceServer ser_cli_target_depth = nh.advertiseService("/fix_abs_depth", service_target_depth);
        ros::ServiceServer ser_cli_target_yaw = nh.advertiseService("/fix_abs_yaw", service_target_yaw);
        ros::ServiceServer ser_cli_target_x = nh.advertiseService("/fix_abs_x", service_target_x);
        ros::ServiceServer ser_cli_target_y = nh.advertiseService("/fix_abs_y", service_target_y);
        ros::ServiceServer ser_cli_target_function = nh.advertiseService("/fix_service" , service_target_function);
    	ros::ServiceServer ser_cli_ok_position = nh.advertiseService("/ok_position" , service_ok_position);
//Pub topic
	 ros::Publisher tell_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
}


//void listen_mode_control(const std_msgs::Int16 message){}

double check_radian_tan(double result){
        if(check < 0)
}
void test_current_state(const geometry_msgs::Point message){
	current_position[0] = message.x;
	current_position[1] = message.y;
	current_position[2] = message.z;
}

void test_current_orientation(const zeabus_controller::orientation message){
	current_position[3] = message.roll;
	current_position[4] = message.pitch;
	current_position[5] = message.yaw; 
}

bool service_target_xy(zeabus_controller::fix_rel_xy::Request &request, zeabus_controller::fix_rel_xy::Response &response){
	target_position[0] = request.x;
	target_position[1] = request.y;
	response.success = true;
	return true;
}
/*
bool srrvice_target_distance(zeabus_controller::fix_rel_xy::Request &request, zeabus_controller::fix_rel_xy::Response &response){
        target_position[0] = request.distance_x*cos(target_position[5]);
        target_position[1] = request.distance_x*sin(target_position[5]);
        target_position[0] = request.distance_y*cos(target_position[5] + (2/PI));
        target_position[1] = request.distance_y*cos(target_position[5] + (2/PI));
        response.success = true;
        return true;
}*/

bool service_target_depth(zeabus_controller::fix_aps_depth::Request &request, zeabus_controller::fix_abs_depth::Response &response){
        target_position[2] = request.fix_depth;
        response.success = true;
        return true;
}        

bool service_target_yaw(zeabus_controller::fix_abs_yaw::Request &request, zeabus_controller::fix_abs_yaw::Response &response){
        target_position[5] = check_radian_tan(request.fix_yaw);
        response.success = true;
        return true;
}
bool service_target_x(zeabus_controller::fix_abs_x::Request &request, zeabus_controller::fix_abs_x::Response &response){
        target_position[0] = request.fix_x;
        response.success = true;
        return true;
}
bool service_target_y(zeabus_controller::fix_abs_y::Request &request, zeabus_controller::fix_abs_y::Response &response){
        target_position[1] = request.fix_y;
        response.success = true;
        return = true;
}
