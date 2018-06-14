// this file will have for help manage file to easy read

#include	<iostream>
#include	<string>
#include	<sstream>
#include 	<ctime>


namespace convert{
	std::string double_to_string( double problem){
		std::ostringstream temporary;
		temporary << problem;
		return temporary.str();
	}

	std::string integer_to_string( int problem){
		std::ostringstream temporary;
		temporary << problem;
		return temporary.str();
	}
}

namespace manage_message{
	std::string ok_one_case( std::string name, std::string result,std::string type, double add,
							double target , double current, double error){
		std::string message = "";
		std::time_t time_now = std::time(NULL);
		message = "------ Order by : " + name  
				+ " when "+(std::string) std::asctime(std::localtime( &time_now))
				+ "--- type is " + type + " and add is " +convert::double_to_string( add ) + "\n"
				+ "target is " + convert::double_to_string( target ) + "\n"
				+ "current is " + convert::double_to_string( current ) + "\n"
				+ "error is " + convert::double_to_string( error ) + "\n"
				+ "--- result is " + result + "\n";
		return message;
	}

	std::string ok_two_case( std::string name, std::string result,std::string type, double add,
							double target_01 , double current_01, double error_01,
							double target_02 , double current_02, double error_02){
		std::string message = "";
		message = "------ Order by : " + name  
				+ " when "+(std::string) std::asctime(std::localtime( &time_now))
				+ "--- type is " + type + " and add is " +convert::double_to_string( add ) + "\n"
				+ "target_01 is " + convert::double_to_string( target_01 ) + "\n"
				+ "current_01 is " + convert::double_to_string( current_01 ) + "\n"
				+ "error_01 is " + convert::double_to_string( error_01 ) + "\n"
				+ "target_02 is " + convert::double_to_string( target_02 ) + "\n"
				+ "current_02 is " + convert::double_to_string( current_02 ) + "\n"
				+ "error_02 is " + convert::double_to_string( error_02 ) + "\n"
				+ "--- result is " + result + "\n";
		return message;
	}

	std::string ok_three_case( std::string name,std::string result, std::string type,double add,
							double target_01 , double current_01, double error_01,
							double target_02 , double current_02, double error_02,
							double target_03 , double current_03, double error_03){
		std::string message = "";
		message = "------ Order by : " + name  
				+ " when "+(std::string) std::asctime(std::localtime( &time_now))
				+ "--- type is " + type + " and add is " +convert::double_to_string( add ) + "\n"
				+ "target_01 is " + convert::double_to_string( target_01 ) + "\n"
				+ "current_01 is " + convert::double_to_string( current_01 ) + "\n"
				+ "error_01 is " + convert::double_to_string( error_01 ) + "\n"
				+ "target_02 is " + convert::double_to_string( target_02 ) + "\n"
				+ "current_02 is " + convert::double_to_string( current_02 ) + "\n"
				+ "error_02 is " + convert::double_to_string( error_02 ) + "\n"
				+ "target_03 is " + convert::double_to_string( target_03 ) + "\n"
				+ "current_03 is " + convert::double_to_string( current_03 ) + "\n"
				+ "error_03 is " + convert::double_to_string( error_03 ) + "\n"
				+ "--- result is " + result + "\n";
		return message;
	}
}

// this is exmample code

/*int main(){
	double problem;
	std::ostringstream ss;
	std::cin >> problem;
	ss << problem;
	std::cout << problem << "\n";
	std::cout << " test is " << ss.str() << "\n";
}*/
