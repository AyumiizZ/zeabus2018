void listen_baro_odom( const nav_msgs::Odometry message_odometry){
	auv_state.twist.twist.linear.z = ( auv_state.pose.pose.position.z 
					- message_odometry.pose.pose.position.z ) / diff_time;
    if(  (! get_pressure ) || 
                        ( absolute_detect( auv_state.pose.pose.position.z 
                                        - message_odometry.pose.pose.position.z , 0.5) ) ){
	    auv_state.pose.pose.position.z = message_odometry.pose.pose.position.z;
        count_detect_depth = 0;
    }
    else{
        count_detect_depth++;
        if( count_detect_depth > limit_detect_depth){
            auv_state.pose.pose.position.z = message_odometry.pose.pose.position.z;
            count_detect_depth = 0;
        }
    }
    get_pressure = true;
    #ifdef data_02
        std::cout << "Receive pressure data\n";
    #endif
}

void listen_dvl_data( const geometry_msgs::TwistWithCovarianceStamped message_twist){
	auv_state.twist.twist.linear.y = message_twist.twist.twist.linear.y;
	auv_state.twist.twist.linear.x = message_twist.twist.twist.linear.x;
    #ifdef data_02
        std::cout << "Receive dvl data\n";
    #endif
    get_dvl = true;
}

void listen_imu_data( const sensor_msgs::Imu message_orientation){
	tf::Quaternion quaternion(	message_orientation.orientation.x
								, message_orientation.orientation.y
								, message_orientation.orientation.z
								, message_orientation.orientation.w );
	tfScalar R , P , Y;
	tf::Matrix3x3( quaternion ).getRPY( R , P , Y);
	roll = (double) R + PI/2;
	pitch = (double) P;
	yaw = (double) Y + PI;
	tf::Quaternion	test = tf::createQuaternionFromRPY( roll , pitch , yaw);
	auv_state.pose.pose.orientation.x = (double)test[0];
	auv_state.pose.pose.orientation.y = (double)test[1];
	auv_state.pose.pose.orientation.z = (double)test[2];
	auv_state.pose.pose.orientation.w = (double)test[3];
	auv_state.twist.twist.angular = message_orientation.angular_velocity;
    // this part will use about solve problem of imu to same past situation
    auv_state.twist.twist.angular.z = -1 * message_orientation.angular_velocity.z;
    get_imu = true;
    #ifdef data_02
        std::cout << "Receive imu data\n";
    #endif
}

bool absolute_detect(double diff , double problem){
    if( diff > problem) return false;
    else if( diff < -1*problem) return false;
    else return true;
}

bool absolute_check(double problem){
    if(problem >= 0.0001) return true;
    else if(problem <= -0.0001) return true;
    else return false;
}

/*
double add_yaw(double problem , double adding){
    
}

double absolute_domain(double problem){

}
*/
