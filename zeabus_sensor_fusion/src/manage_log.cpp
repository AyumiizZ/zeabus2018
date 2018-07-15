#include    <fstream>
#include    <iostream>
#include    <ctime>
#include    <string>
#include    <cstdlib>
#include    <sstream>
#include    <ros/package.h>
#define name_file "control_tune"

class manage_log{
	private:
		std::time_t time_now;
        std::time_t start_time
		
	public:
		manage_log(std::string type , std::time_t start_time);
		std::string sub_directory;
		std::string locate_file;
		std::string cmd_string;
		void write_log(std::string message);
};

	manage_log::manage_log(std::string type ,  std::time_t start_time){
		start_time = std::time(NULL);
		sub_directory = ros::package::getPath("zeabus_controller") + "/src/log";
		locate_file = "2018_06_19";
		cmd_string = "echo \"----------------- start log control -----------------\" > " 
					 + sub_directory + "/" + locate_file + ".txt";
		std::system( cmd_string.c_str() );
	}

	void manage_log::write_log( std::string message){
		time_now = std::time(NULL);
		cmd_string = "echo \"------------------ at " 
					 + (std::string) std::asctime(std::localtime(&time_now)) 
					 + " >> " + sub_directory + "/" + locate_file + ".txt";
		std::system( cmd_string.c_str() );
		cmd_string = "echo \"" + message + "\"" 
					 + " >> " + sub_directory + "/" + locate_file + ".txt";
		std::system( cmd_string.c_str() );
	}
