#include	<iostream>
#include	<cstdlib>
#include	<ros/package.h>
#include	<ctime>
#include	<fstream>

class order_log{
	private:
		std::string name_file;
		std::string adding_path;
		std::string package_path;
		std::string location_file;

	public:
		order_log( str::string name_file );
		void save_log( str::string data);

};

	order_log::order_log( str::string name_file , str::string package , str::string adding){
		this->name_file = name_file
		this->adding_path = adding;
		this->package_path = ros::package::getPath(" package ");
		this->location_file = this->package_path + "/"
							 + this->adding_path + "/"
							 + this->name_file;
		std::cout << "Intial file : " << this->location_file << std::endl;
		
		std::string cmd_string = "echo " + "\"---- start log ----\" > " 
								+ this->name_file + ".txt";
		std::system( cmd_string.c_str() );
	}

	order_log::save_log( str::string data){
		std::string cmd_string = "echo " + "\"" + data + "\" >> " + this->name_file + ".txt";
		std::system( cmd_string.c_str() );
	}