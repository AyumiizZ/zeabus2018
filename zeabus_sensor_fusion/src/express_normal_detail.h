void listen_baro_odom( const nav_msgs::Odometry message_odometry){
	auv_state.twist.twist.linear.z = ( auv_state.pose.pose.position.z 
					- message_odometry.pose.pose.position.z ) / diff_time;
	auv_state.pose.pose.position.z = message_odometry.pose.pose.position.z;
	get_pressure = true; 
}
void listen_dvl_data( const geometry_msgs::TwistWithCovarianceStamped message_twist){
	auv_state.twist.twist.linear.y = message_twist.twist.twist.linear.y;
	auv_state.twist.twist.linear.x = message_twist.twist.twist.linear.x;
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
	yaw = (double) Y;
	tf::Quaternion	test = tf::createQuaternionFromRPY( roll , pitch , yaw);
	auv_state.pose.pose.orientation.x = (double)test[0];
	auv_state.pose.pose.orientation.y = (double)test[1];
	auv_state.pose.pose.orientation.z = (double)test[2];
	auv_state.pose.pose.orientation.w = (double)test[3];
	get_imu = true; 
}
