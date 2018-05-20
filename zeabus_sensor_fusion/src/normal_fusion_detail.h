void fusion_depth_velocity_imu( const nav_msgs::Odometry message_odometry, 
								const geometry_msgs::TwistWithCovarianceStamped message_twist ,
								const sensor_msgs::Imu message_orientation){
	tf::Quaternion quaternion(	message_orientation.orientation.x
								, message_orientation.orientation.y
								, message_orientation.orientation.z
								, message_orientation.orientation.w );
	tfScalar roll , pitch , yaw;
	tf::Matrix3x3( quaternion ).getRPY( roll , pitch ,yaw);

	current_time = ros::Time::now();

	#ifdef data_01
		std::cout << "message_come_form_baro_odom : "
				  << message_odometry.pose.pose.position.z
				  << "\n";
	#endif	

	#ifdef data_01
		std::cout << "message_come_form_dvl_data : "
				  << message_twist.twist.twist.linear.x << " , "
				  << message_twist.twist.twist.linear.y << " , "
				  << message_twist.twist.twist.linear.z << " \n ";
	#endif

	#ifdef data_01
		std::cout << "message_come_form_imu_data : "
				  << roll << " , " << pitch <<  " , " << yaw << "\n";
		std::cout << "message_twist_angular_form_imu_data : "
				  << message_orientation.angular_velocity.x << " , "
				  << message_orientation.angular_velocity.y << " , "
				  << message_orientation.angular_velocity.z << "\n";
	#endif

	if( start_run ){
		previous_time = current_time;
		previous_velocity_x = message_twist.twist.twist.linear.x;
		previous_velocity_y = message_twist.twist.twist.linear.y;
		previous_depth = message_odometry.pose.pose.position.z;
		#ifdef data_01
			std::cout << "---------------- finish init first time --------------------\n";
		#endif
	}
	else{
		double diff_time = (current_time - previous_time).toSec();
		add_robot_x = ( previous_velocity_x + message_twist.twist.twist.linear.x) 
							/ 2 * diff_time * cos(pitch);
		add_robot_y = ( previous_velocity_y + message_twist.twist.twist.linear.y) 
							/ 2 * diff_time * cos(roll);
		add_world_x = add_robot_x * cos( yaw ) + add_robot_y * cos( yaw + PI/2 );
		add_world_y = add_robot_x * sin( yaw ) + add_robot_y * sin( yaw + PI/2 );
		world_x += add_world_x;
		world_y += add_world_y;
		#ifdef data_02
			ROS_INFO("----------------------------print data-------------------------------\n");
			ROS_INFO("previous_data:\t%.2lf\t%.2lf\t%.2lf\n" 
					 	, msgs_auv_state.pose.pose.position.x 
						, msgs_auv_state.pose.pose.position.y
						, msgs_auv_state.pose.pose.position.z);
			ROS_INFO("adding_robot:\t%.2lf\t%.2lf\n"
						, add_robot_x , add_robot_y);	 
			ROS_INFO("adding_world:\t%.2lf\t%.2lf\n"
						, add_world_x , add_world_y);	 
			ROS_INFO("current_pos :\t%.2lf\t%.2lf\n"
						, world_x , world_y);
		#endif
// form dvl and Imu
		msgs_auv_state.pose.pose.position.x = world_x;
		msgs_auv_state.pose.pose.position.y = world_y;
// form pressure sensor
		msgs_auv_state.pose.pose.position.z = message_odometry.pose.pose.position.z;
// from Imu
		msgs_auv_state.pose.pose.orientation = message_orientation.orientation;
// from dvl
		msgs_auv_state.twist.twist.linear.x = message_twist.twist.twist.linear.x;
		msgs_auv_state.twist.twist.linear.y = message_twist.twist.twist.linear.y;
// from pressure sensor
		msgs_auv_state.twist.twist.linear.z = ( message_odometry.pose.pose.position.z 
											  - previous_depth ) / diff_time;
// from Imu
		msgs_auv_state.twist.twist.angular = message_orientation.angular_velocity;
		
		msgs_auv_state.header.stamp = ros::Time::now();

		tell_auv_state.publish( msgs_auv_state );
	}
	
}
