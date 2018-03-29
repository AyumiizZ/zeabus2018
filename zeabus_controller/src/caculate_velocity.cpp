// this include standare operation of c++
#include 	<iostream>

//	this include for use time of ros
#include 	<ros/ros.h>

/////////////////////////////////////////////////
//       -----Supasan Komonlit 2018-----
// This namespace using find target velocity
// That have many way to find that
// In first case :
//		I consider to find what good velocity when 
//		force from thruster is zero and zero when 
//		distance / 2
// In second case :
//		I find to use tunnig find what velocity
//		We have to want that
/////////////////////////////////////////////////

namespace find_velocity{

	const static double default_ttl = 1.0;

// --------------------------------------------------------------------------------------- //

// for first case we will use equation is distance 	= k ln ( kv{zero} / v) 
//													from v = v{zero} to v{final}
// 												   	= k  ln(kv{zero}/v{final}) -ln(k)
//													= k ln(kv{zero}/kv{final})
//													= k ln(v{zero}/v{final})
//												v	= e^(distance/k) / v{final}
//	so v{final} is should equal 0 but we will use 0.01 for estimate that
//  k is constant

	class first_case{
		protected:
			double constant;
			double final_velocity;
		public:
			first_case(double constant , double final_velocity);
			double calculate_velocity(double error_distance);
			void set_constant(double constant)
	}
// declare detail of init class or object
// You must to declare about constant and final_velocity already
	first_case::first_case(double constant , double final_velocity){
		std::cout << "Init first case about find_velocitiy" << std::endl;
		std::cout 	<< "Input value for K is" << constant 
 					<< " and value for final_velocity is " << final_velocity << std::endl;
		this->constant = constant;
		this->final_velocity = final_velocity;
	}

// this function for calculate target_velocity	 
	double first_case::calculate_velocity(double error_distance , double constant){
		return std::exp( error_distance / constant ) / final_velocity;
	}	

// set up new constant k of this object
	void first_case::set_constant(double constant){
		this->constant = constant;
	}
// --------------------------------------------------------------------------------------- //

// this second case I will use tuning to find velocity
// I will use equation from PID
// if you use this equation that mean you can't set force from thruster is 0
// That mean P is poportional , I is Intergral , D is defferential
	class second_case{
		private:
			ros::Time previous_time;
			double sum_error;
			double previous_error;
			double diff_time;
			bool use_ttl;

		protected:
			double P_constant;
			double I_constant;
			double D_constant;
			
		public:
			second_case(double P_constant , double I_constant , double D_constant);
			double calculate_velocity(double error_distance);
			void set_constant(double P_constant , double I_constant , double D_constant);
			void reset();	
	}

// Init second case and assign value of P I D constant
	second_case::second_case(double P_constant, double I_constant, double D_constant){
		std::cout << "Init second case about find_velocitiy" << std::endl;
		this->P_constant = P_constant;
		this->I_constant = I_constant;
		this->D_constant = D_constant;
	}

}
