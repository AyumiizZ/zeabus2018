#include	<ros/ros.h> // Use ros system
#include	<ros/time.h> // Use time by ros system

#include 	<iostream> // Base of CPP Language

#include 	"manage_file.cpp" // Use Load or Save dynamic value

// 2 line will use between quaternion with roll pitch yaw
#include 	<tf/transform_datatypes.h>
#include 	<tf/transform_listener.h>

#include	<nav_msgs/Odometry.h> // Include message for receive auv_state
#include 	<geometry_msgs/Twist.h> // Include message for send to thruster_mapper

#include 	<std_msgs/Float64.h> // Include message for receive or send variable type float 64
#include 	<std_msgs/String.h> // Include message for receive type string


