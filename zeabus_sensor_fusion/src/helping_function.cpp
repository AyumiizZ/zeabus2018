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

