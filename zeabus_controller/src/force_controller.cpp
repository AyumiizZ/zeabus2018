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
#include	<zeabus_controller/point_xy.h>
#include	<modbus_ascii_ros/Switch.h>
//#include	<dynamic_reconfigure/server.h>
//#include	<zeabus_controller/PIDConstantConfig.h>

//#include	<zeabus_controller/drive_x.h>
//#include	<zeabus_controller/message_service.h>
#include	<zeabus_controller/fix_abs_xy.h>
#include	<zeabus_controller/fix_abs_x.h>
#include	<zeabus_controller/fix_abs_y.h>
#include	<zeabus_controller/fix_abs_depth.h>
#include	<zeabus_controller/fix_abs_yaw.h>
#include	<zeabus_controller/fix_rel_xy.h>
#include	<zeabus_controller/ok_position.h>
