void listen_current_state( const nav_msgs::Odometry message){
	tf::Quaternion quaternion( message.pose.pose.orientation.x,
								message.pose.pose.orientation.y,
								message.pose.pose.orientation.z,
								message.pose.pose.orientation.w			
							);
	tfScalar roll, pitch, yaw;
	tf::Matrix3x3(quaternion).getRPY( roll, pitch, yaw);
	if( start_run || reset_position ){
		if( not ( want_fix[0] && want_fix[1] ) || start_run ){
			target_position[0] = message.pose.pose.position.x;
			target_position[1] = message.pose.pose.position.y;
		}
		if( ( not want_fix[2] ) || start_run ){
			target_position[2] = message.pose.pose.position.z;
		}
		if( ( not want_fix[3] )){
			target_position[3] = 0.0;
		}
		if( ( not want_fix[4] )){
			target_position[4] = 0.0;
		}
		if( ( not want_fix[5] ) || start_run ){
			target_position[5] = convert_range_radian( bound_value_radian((double) yaw));
		}
		start_run = false;
		reset_position = false;
	}
	current_position[0] = message.pose.pose.position.x;
	current_position[1] = message.pose.pose.position.y;
	current_position[2] = message.pose.pose.position.z;
	current_position[3] = convert_range_radian( bound_value_radian((double) roll));
	current_position[4] = convert_range_radian( bound_value_radian((double) pitch));
	current_position[5] = convert_range_radian( bound_value_radian((double) yaw));
	current_velocity[0] = message.twist.twist.linear.x;
	current_velocity[1] = message.twist.twist.linear.y;
	current_velocity[2] = message.twist.twist.linear.z;
	current_velocity[3] = message.twist.twist.angular.x;
	current_velocity[4] = message.twist.twist.angular.y;
	current_velocity[5] = message.twist.twist.angular.z;
}

void listen_target_velocity( const geometry_msgs::Twist message){
	target_velocity[0] = message.linear.x;
	target_velocity[1] = message.linear.y;
	target_velocity[2] = message.linear.z;
	target_velocity[3] = message.angular.x;
	target_velocity[4] = message.angular.y;
	target_velocity[5] = message.angular.z;
	last_target_velocity = ros::Time::now();
	for(int count = 0 ; count < 6 ; count++){
		if( absolute( target_velocity[ count ] )> epsilon){
			#ifdef test_02
				std::cout << "want fix of " << count << " will false\n";
			#endif
			want_fix[count] = false;
		}
	}
}

double convert_min_radian( double problem){
	for( ; ;){
		#ifdef test_02
			std::cout << "find min radian now is " << problem << "\n";
		#endif
		if( problem > PI) problem -= 2*PI;
		else if(problem < -1*PI) problem += 2*PI;
		else break;
	}
	#ifdef test_02
		std::cout << "result is " << problem << "\n";
	#endif
	return problem;
}

double convert_range_radian( double problem){
	if( problem < 0 ) return problem + 2*PI;
	else return problem;
}

double bound_value_radian( double problem){
	for( ; ;){
		#ifdef test_02
			std::cout << "bound radian now is " << problem << "\n";
		#endif
		if( problem < 0) problem += 2*PI;
		else if(problem > 2*PI) problem -= 2*PI;
		else break;
	}
	#ifdef test_02
		std::cout << "result is " << problem << "\n";
	#endif
	return problem;
}

void config_constant_PID( zeabus_controller::OffSetConstantConfig &config,	uint32_t level){
	#ifdef print_data
		ROS_INFO("!!!---tunnig change---!!!");
	#endif
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
	Kd_position[5] = config.KDPyaw;

	Kp_velocity[0] = config.KPVx;
	Kp_velocity[1] = config.KPVy;
	Kp_velocity[2] = config.KPVz;
	Kp_velocity[3] = config.KPVroll;
	Kp_velocity[4] = config.KPVpitch;
	Kp_velocity[5] = config.KPVyaw;

	Ki_velocity[0] = config.KIVx;
	Ki_velocity[1] = config.KIVy;
	Ki_velocity[2] = config.KIVz;
	Ki_velocity[3] = config.KIVroll;
	Ki_velocity[4] = config.KIVpitch;
	Ki_velocity[5] = config.KIVyaw;

	Kd_velocity[0] = config.KDVx;
	Kd_velocity[1] = config.KDVy;
	Kd_velocity[2] = config.KDVz;
	Kd_velocity[3] = config.KDVroll;
	Kd_velocity[4] = config.KDVpitch;
	Kd_velocity[5] = config.KDVyaw;

	K_velocity[0] = config.KVx;
	K_velocity[1] = config.KVy;
	K_velocity[2] = config.KVz;

	offset_force[0] = config.OFFSETx;
	offset_force[1] = config.OFFSETy;
	offset_force[2] = config.OFFSETz;
	offset_force[3] = config.OFFSETroll;
	offset_force[4] = config.OFFSETpitch;
	offset_force[5] = config.OFFSETyaw;

	#ifdef test_02
		ROS_INFO("!!!--- Change tune value ---!!!");
		ROS_INFO("--- Offset of z %.4lf", offset_force[2]);
	#endif
	if( not first_time_tune){
		change_tune = true;
	}
}

// ------------------------------- this part about service ------------------------------------

bool service_target_distance(
		zeabus_controller::fix_rel_xy::Request &request , 
		zeabus_controller::fix_rel_xy::Response &response){
	target_position[0] += request.distance_x * cos( target_position[5]);
	target_position[1] += request.distance_y * sin( target_position[5]);
	target_position[0] += request.distance_x * cos( target_position[5] + PI/2);
	target_position[1] += request.distance_y * sin( target_position[5] + PI/2);
	response.success = true;
	return true;
}

bool service_target_yaw(
		zeabus_controller::fix_abs_yaw::Request &request ,
		zeabus_controller::fix_abs_yaw::Response &response){
	target_position[5] = convert_range_radian( request.fix_yaw );
	response.success = true;
	return true;
}

bool service_target_depth(
		zeabus_controller::fix_abs_depth::Request &request , 
		zeabus_controller::fix_abs_depth::Response &response){
	target_position[2] = request.fix_depth;
	response.success = true;
	return true;
}

bool service_change_mode(
		zeabus_controller::change_mode::Request &request , 
		zeabus_controller::change_mode::Response &response){
	mode_control = request.mode.data;
	#ifdef test_02
		std::cout << "change mode" << "\n";
	#endif
	response.success.data = true;
	return true;
}

bool service_ok_position(
		zeabus_controller::ok_position::Request &request ,
		zeabus_controller::ok_position::Response &response){
	#ifdef test_02
	    std::cout << "service check " << request.type.data 
					<< "and use adding is " << request.adding << std::endl;
	#endif
    if(request.type.data == "xy"){
		#ifdef print_data
			std::cout 	<< "------------------ check position "
		 				<< "want to know ok xy--------------\n";
			std::cout 	<< "for x : robot " << robot_error[0]
						<< " ok_error + adding " << ok_error[0] + request.adding << "\n";
			std::cout 	<< "for y : robot " << robot_error[1]
						<< " ok_error + adding " << ok_error[1] + request.adding << "\n";
		#endif
        if( absolute(robot_error[0]) < ok_error[0] + request.adding 
			&& absolute(robot_error[1]) < ok_error[1] + request.adding) response.ok = true;
        else response.ok = false;
    }
    else if(request.type.data == "z"){
		#ifdef print_data
			std::cout 	<< "------------------ check position "
		 				<< "want to know ok z--------------\n";
			std::cout 	<< "for z : robot " << robot_error[2]
						<< " ok_error + adding " << ok_error[2] + request.adding << "\n";
		#endif
        if( absolute(robot_error[1]) < ok_error[2] + request.adding) response.ok = true;
        else response.ok = false;
    }
	else if(request.type.data == "xyz"){
		#ifdef print_data
			std::cout 	<< "------------------ check position "
		 				<< "want to know ok xyz--------------\n";
			std::cout 	<< "for x : robot " << robot_error[0]
						<< " ok_error + adding " << ok_error[0] + request.adding << "\n";
			std::cout 	<< "for y : robot " << robot_error[1]
						<< " ok_error + adding " << ok_error[1] + request.adding << "\n";
			std::cout 	<< "for z : robot " << robot_error[2]
						<< " ok_error + adding " << ok_error[2] + request.adding << "\n";
		#endif
		if( absolute(robot_error[0]) < ok_error[0] + request.adding
			&& absolute(robot_error[1]) < ok_error[1] + request.adding
			&& absolute(robot_error[2]) < ok_error[2] + request.adding) response.ok = true;
		else response.ok = false;
	}
	else if(request.type.data == "yaw"){
		#ifdef print_data
			std::cout 	<< "------------------ check position "
		 				<< "want to know ok yaw--------------\n";
			std::cout 	<< "for z : robot " << robot_error[5]
						<< " ok_error + adding " << ok_error[5] + request.adding << "\n";
		#endif
		if( absolute(robot_error[5]) < ok_error[5] + request.adding) response.ok = true;
		else response.ok = false;
	}
    else response.ok = false;
	#ifdef test_02
    	std::cout << "Result is " << response.ok << std::endl;
	#endif
    return true;
	
}
// ------------------------------------- end part ---------------------------------------------

// --------------- this part will control about value of tunning except offset ----------------
void reset_all_I(){
	for(int count = 0 ; count < 6 ; count++){
		PID_position[count].reset();
		PID_velocity[count].reset();	
	}
}

void set_all_tunning(){
	for(int count = 0 ; count < 6 ; count++){
		PID_position[count].set_constant( 	Kp_position[count],
											Ki_position[count],
											Kd_position[count]);
		PID_velocity[count].set_constant(	Kp_velocity[count],
											Ki_velocity[count],
											Kd_velocity[count]);
	}
}

void reset_specific_position( int number){
	PID_position[ number ].reset();
}

void reset_specific_velocity( int number){
	PID_velocity[ number ].reset();
}

// -------------------------------------- end part --------------------------------------------

double absolute( double problem){
	if(problem < 0) return -1*problem;
	else return problem;
}

geometry_msgs::Twist create_msg_force(){
	geometry_msgs::Twist message;
	message.linear.x = sum_force[0];
	message.linear.y = sum_force[1];
	message.linear.z = sum_force[2];
	message.angular.x = sum_force[3];
	message.angular.y = sum_force[4];
	message.angular.z = sum_force[5];
	return message;
	
}

void reset_want_fix(){
	for(int count = 0 ; count < 6 ; count++) want_fix[count] = true;
}

#ifdef test_01
	void test_current_state( const geometry_msgs::Point message){
		current_position[0] = message.x;
		current_position[1] = message.y;
		current_position[2] = message.z;
}

	void test_current_orientation( const zeabus_controller::orientation message){
		current_position[3] = message.roll;
		current_position[4] = message.pitch;
		current_position[5] = message.yaw;
}
#endif
