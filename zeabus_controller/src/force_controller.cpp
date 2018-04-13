#include	<ros/ros.h> // Use ros system
#include	<ros/time.h> // Use time by ros system

#include 	<iostream> // Base of CPP Language
#include 	<> 

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
#include	<zeabus_controller/point_xy.h>
