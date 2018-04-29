// please :set nu tabstop=4

// include 2 part file
#include "offset_controller_header.h"
#include "offset_controller_detail.h"

// initial code run 1 time
void init(){
	#ifdef test_02
		std::cout << "welocome to init file\n";
	#endif
	PID_position = ( find_velocity::second_case)calloc 
						(6 , sizeof( find_velocity::second_case));
	PID_velocity = ( find_velocity::second_case)calloc
						(6 , sizeof( find_velocity::second_case));
	for(int count = 0 ; count < 6 count++){
		PID_position[count].set_constant(0 ,0 ,0);
		PID_velocity[count].set_constant(0 ,0 ,0);
	}
}

int main(int argc , char **argv){
// declare basic ros subscribe . publish . service
	ros::init(argc , argv, "Controller");
	ros::NodeHandle nh;

// ---------------------------------- part of subscriber --------------------------------------
	ros::Subscriber sub_state = // listen now where I am
		nh.subscribe( "/auv/state" , 1000 , &listen_current_state);
	ros::Subscriber sub_target_velocity = // listen what target do you want
		nh.subscribe( "/zeabus/cmd_vel" , 1000 , &listen_target_velocity);
// ------------------------------------- end part ---------------------------------------------

// ---------------------------------- part of service -----------------------------------------
	ros::ServiceServer ser_cli_target_distance = // listen target of xy
		nh.advertiseService("/fix_rel_xy", service_target_xy);
	ros::ServiceServer ser_cli_target_yaw = // listen target of yaw
		nh.advertiseService("/fix_abs_yaw", service_target_yaw);
	ros::ServiceServer ser_cli_target_depth = // listen target of depth
		nh.advertiseService("/fix_abs_depth", service_target_depth);
	ros::ServiceServer ser_cli_ok_position = // listen and answer now ok?
		nh.advertiseService("/ok_position", service_ok_position);
	ros::ServiceServer ser_cli_change_mode = // listen target mode
		nh.advertiseService("/change_mode", service_change_mode); 
// ------------------------------------- end part ---------------------------------------------

// ------------------------------------ test state --------------------------------------------
	#ifdef test_01
		ros::Subscriber sub_test_state = // listen test state
			nh.Subscriber( "/test/auv/state" , 1000 , &test_current_state);
		ros::Subscriber sub_test_orientation = // listen test orientation
			nh.Subscriber( "/test/auv/orientation" , 1000 , &test_current_orientation);
	#endif 
// ------------------------------------- end part ---------------------------------------------

// ----------------------------------- part of publisher --------------------------------------
	ros::Publisher tell_force = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
// -------------------------------------- end part --------------------------------------------

// ------------------------------- part of dynamic reconfiure ---------------------------------
	dynamic_reconfigure::Server<zeabus_controller::OffSetConstantConfig> server;
	dynamic_reconfigure::Server<zeabus_controller::OffSetConstantConfig>::CallbackType tunning;
// -------------------------------------- end part --------------------------------------------

	init();

// ---------------------------------- again about dynamic -------------------------------------
	tunning = boost::bing(&config_constant_PID, _1 , _2);
	server.setCallback( tunning );
// -------------------------------------- end part --------------------------------------------

	ros::Rate rate(50);
	while(nh.ok()){
		if(first_time_tune){
			#ifdef test_02
				std::cout << "Before download\n";
			#endif
				PID_file.load_file("Controller");
			#ifdef test_02
				std::cout << "After download\n";
			#endif
			first_time_tune = false;
			rate.sleep();
			set_all_tunning();
			reset_all_I();
		}
		else if(change_tune){
			#ifdef test_02
				std::cout << "Before save\n";
			#endif
				PID_file.save_file("Controller");
			#ifdef test_02
				std::cout << "After savw\n";
			#endif
			change_tune = false;
			set_all_tunning();
			reset_all_I();
		}
	}
}
