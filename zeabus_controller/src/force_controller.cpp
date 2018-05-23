#include <ros/ros.h> // Use ros system
#include <ros/time.h> // Use time by ros system

#include <iostream> // Base of CPP Language
#include <stdlib.h> 
#include <vector>
#include <cmath>
#include <string>
#include <Vector3>
#include <queue> 

#include "manage_file.cpp" // Use Load or Save dynamic reconfigure
#include "calculate_force.cpp" // Use calculate force form acceleration
#include "new_PID.cpp"

// 2 line will use between quaternion with roll pitch yaw
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h> // Include message for receive auv_state
#include <geometry_msgs/Twist.h> // Include message for send to thruster_mapper
#include <geometry_msgs/Point.h> 
#include <geometry_msgs/Pose.h> 
#include <std_msgs/Float64.h> // Include message for receive or send variable type float 64
#include <std_msgs/Float32.h> // Include message for receive or send variable type float 32
#include <std_msgs/Int16.h> // Include message for receive type int
#include <std_msgs/String.h> // Include message for receive type string
#include <std_msgs/Bool.h> // Include message for receive type bool
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <zeabus_controller/point_xy.h>
#include <zeabus_controller/orientation.h>
#include <modbus_ascii_ros/Switch.h>
#include <dynamic_reconfigure/server.h>
#include <zeabus_controller/PIDConstantConfig.h>

//#include <zeabus_controller/drive_x.h> //unused in code
#include <zeabus_controller/message_service.h>
#include <zeabus_controller/fix_abs_xy.h>
#include <zeabus_controller/fix_abs_x.h>
#include <zeabus_controller/fix_abs_y.h>
#include <zeabus_controller/fix_abs_depth.h>
#include <zeabus_controller/fix_abs_yaw.h>
#include <zeabus_controller/fix_rel_xy.h>
#include <zeabus_controller/ok_position.h>

#define PI 3.1415926535

//void listen_mode_control(const std_msgs::Int16 message);
//for testing
void test_current_state(const geometry_msgs::Point message);
void test_current_orientation(const zeabus_controller::orientation message);

//setup subscribe
void listen_current_state(const nav_msgs::Odometry message);
void listen_target_velocity(const geometry_msgs::Twist message);
void listen_target_position(const geometry_msgs::Point message);
void listen_target_depth(const std_msgs::Float64 message);
void listen_absolute_yaw(const std_msgs::Float64 message);
void listen_real_yaw(const std_msgs::Float64 message);
void listen_absolute_xy(const zeabus_controller::point_xy message);
void listen_absolute_orientation(const zeabus_controller::orientation message);

//about dynamic reconfigure
void config_constant_PID(zeabus_controller::PIDConstantConfig &config, unit32_t level);

//setup function of service
bool service_target_xy(zeabus_controller::fix_abs_xy::Request &request, zeabus_controller::fix_abs_xy::Response &response);
bool service_target_distance(zeabus_controller::fix_rel_xy::Request &request, zeabus_controller::fix_rel_xy::Response &response);
bool service_target_z(zeabus_controller::fix_rel_depth::Request &request, zeabus_controller::fix_rel_depth::Response &response);
bool service_target_depth(zeabus_controller::fix_abs_depth::Request &request, zeabus_controller::fix_abs_depth::Response &response);
bool service_target_yaw(zeabus_controller::fix_abs_yaw::Request &request, zeabus_controller::fix_abs_yaw::Response &response);
bool service_target_x(zeabus_controller::fix_abs_x::Request &request, zeabus_controller::fix_abs_x::Response &response);
bool service_target_y(zeabus_controller::fix_abs_y::Request &request, zeabus_controller::fix_abs_y::Response &response);
//bool service_target_function(zeabus_controller::message_service::Request &request, zeabus_controller::message_service::Response &response);
//bool service_ok_position(zeabus_controller::ok_position::Request &request, zeabus_controller::ok_position::Response &response);

//about function in code
double check_tan_radian(check);
double change_pi_radian(double value);

//setup bool
bool start_run = true;
bool reset_position = true;

PID *PID_position, *PID_velocity;

void init(){
        PID_position = (PID*)calloc(6, sizeof(PID));
        PID_velocity = (PID*)calloc(6, sizeof(PID));
        for(int num = 0 ; num < 6 ; num ++){ 
            PID_position[num].set_PID(Kp_position[num], Ki_position[num], Kd_position[num], Kvs_position[num]);
            PID_velocity[num].set_PID(Kp_velocity[num], Ki_position[num], Kd_position[num], 0); 
            PID_position.reset_I();
            PID_velocity.reset.I();
    }
}

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
        ros::ServiceServer ser_cli_target_z = nh.advertiseService("/fix_rel_depth", service_target_z);
        ros::ServiceServer ser_cli_target_depth = nh.advertiseService("/fix_abs_depth", service_target_depth);
        ros::ServiceServer ser_cli_target_yaw = nh.advertiseService("/fix_abs_yaw", service_target_yaw);
        ros::ServiceServer ser_cli_target_x = nh.advertiseService("/fix_abs_x", service_target_x);
        ros::ServiceServer ser_cli_target_y = nh.advertiseService("/fix_abs_y", service_target_y);
        ros::ServiceServer ser_cli_target_function = nh.advertiseService("/fix_service" , service_target_function);
    	ros::ServiceServer ser_cli_ok_position = nh.advertiseService("/ok_position" , service_ok_position);
//Pub topic
	 ros::Publisher tell_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
//setup PID
        dynamic_reconfigure::Server<zeabus_controller::PIDConstantConfig> server;
        dynamic_reconfigure::Server<zeabus_controller::PIDConstantConfig>::CallbackType tune;
        
        init();

        tune = boost::bind(config_constant_PID, _1, _2);
        server.setCallback(tune);

        ros::Rate rate(50);

//void listen_mode_control(const std_msgs::Int16 message){}

double check_tan_radian(double value){ 
        if(value < 0) return value + 2*PI;
        else if(value > 2*PI) return value - 2*PI; 
        else return value;
}

double change_pi_radian(double value){
        if(value > PI) return value - 2*PI;
        else return value;

void listen_current_state(const nav_msgs::Odometry message){
        tf::Quaternion quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w);
        tfScalar roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getPRY(roll, pitch, yaw);
        if(start_run || reset_position){
            target_position[0] = message.pose.pose.position.x;
            target_position[1] = message.pose.pose.position.y;
            target_position[2] = message.pose.pose.position.z;
            target_position[3] = 0.0
            target_position[4] = 0.0
            target_position[5] = check_tan_radian(double yaw);
            start_run = false;
            reset_position = false;
            }
        current_position[0] = message.pose.pose.position.x;
        current_position[1] = message.pose.pose.position.y;
        current_position[2] = message.pose.pose.position.z;
        current_position[3] = check_tan_radian(double roll);
        current_position[4] = check_tan_radian(double pitch);
        current_position[5] = check_tan_radian(double yaw);
        current_velosity[0] = message.twist.twist.linear.x;
        current_velosity[1] = message.twist.twist.linear.y;
        current_velosity[2] = message.twist.twist.linear.z;
        current_velosity[3] = message.twist.twist.angular.x;
        current_velosity[4] = message.twist.twist.angular.y;
        current_velosity[5] = message.twist.twist.angular.z;
}

void listen_target_velocity(const geometry_msgs::Twist message){
        target_velocity[0] = message.linear.x; 
        target_velocity[1] = message.linear.y; 
        target_velocity[2] = message.linear.z; 
        target_velocity[3] = message.angular.x;
        target_velocity[4] = message.angular.y;
        target_velocity[5] = message.angular.z;
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

void listen_target_depth(const std_msgs::Float64 message){
        target_position[2] = message.data;
}

void config_constant_PID(zeabus_controller::PIDConstantConfig &config, unit32_t level){
        ROS_INFO("!!--------K change--------!!")
        Kp_position[0] = config.KPPx;
        Kp_position[1] = config.KPPy;
        Kp_position[2] = config.KPPz;
        Kp_position[3] = config.KPProll;
        Kp_position[4] = config.KPPpitch;
        Kp_position[5] = config.KPPyaw;

        Ki_position[0] = config.KIPx;
        Ki_position[1] = config.KIPy;
        Ki_position[2] = config.KIPz;
        Ki_position[3] = config.KIProll;
        Ki_position[4] = config.KIPpitch;
        Ki_position[5] = config.KIPyaw;

        Kd_position[0] = config.KDPx;
        Kd_position[1] = config.KDPy;
        Kd_position[2] = config.KDPz;
        Kd_position[3] = config.KDProll;
        Kd_position[4] = config.KDPpitch;
        Kd_position[5] = config.KDPyoll;

        Kp_velocity[0] = config.KPVx;
        Kp_velocity[1] = config.KPVy;
        Kp_velocity[2] = config.KPVz;
        Kp_velocity[3] = config.KPVroll;
        Kp_velocity[4] = config.KPVpitch;
        Kp_velocity[5] = config.KPVyaw;

        Ki_velosity[0] = config.KIVx;
        Ki_velosity[1] = config.KIVy;
        Ki_velosity[2] = config.KIVz;
        Ki_velosity[3] = config.KIVroll;
        Ki_velosity[4] = config.KIVpitch;
        Ki_velosity[5] = config.KIVyaw;

        Ki_velosity[0] = config.KDVx;
        Ki_velosity[1] = config.KDVy;
        Ki_velosity[2] = config.KDVz;
        Ki_velosity[3] = config.KDVroll;
        Ki_velosity[4] = config.KDVpitch;
        Ki_velosity[5] = config.KDVyaw;

        Kvs_position[0] = config.KVSx;
        Kvs_position[1] = config.KVSy;
        Kvs_position[2] = config.KVSz;
        Kvs_position[3] = config.KVSroll;
        Kvs_position[4] = config.KVSpitch;
        Kvs_position[5] = config.KVSyaw;
}

bool service_target_xy(zeabus_controller::fix_abs_xy::Request &request, zeabus_controller::fix_abs_xy::Response &response){
	target_position[0] = request.x;
	target_position[1] = request.y;
	response.success = true;
	return true;
}

bool service_target_distance(zeabus_controller::fix_rel_xy::Request &request, zeabus_controller::fix_rel_xy::Response &response){
        target_position[0] = request.distance_x*cos(target_position[5]);
        target_position[1] = request.distance_x*sin(target_position[5]);
        target_position[0] = request.distance_y*cos(target_position[5] + (2/PI));
        target_position[1] = request.distance_y*cos(target_position[5] + (2/PI));
        response.success = true;
        return true;
}

bool service_target_z(zeabus_controller::fix_rel_depth::Request &request, zeabus_controller::fix_rel_depth::Response &response){
        target_position[2] += request.fix_depth;
        response.success = true;
        return True;
}

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
