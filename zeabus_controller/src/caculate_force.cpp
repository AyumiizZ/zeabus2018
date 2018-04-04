// please :set nu tabstop=4

#include 	<ros/ros.h> // include if you wan to use time
#include 	<math.h> // use math for calculate

//////////////////////////////////////////////////////////////////////////////////////////
//			-------Supasan Komonlit 2018--------------
// this code will use to calculate what force you want in each axis
// that mean I will use physics to calculate force so
// I have to know about mass to calculate that
// equation is SumOfF = ma
//		then SumOfF is F{thruster} + F{gravity} + F{drag or liquid} + F{nature}
//		we have decide about sign of variable by right hand rules
// I accept about my knowledge have few knowledge to use this solution
// but I think that is one way to control about velocity and have 
// constant force will want to control abot roll pitch in the future
/////////////////////////////////////////////////////////////////////////////////////////

const static double mass;

// normal from is F(thruster) F(gravity) F(drag of liquid)

namespace find_force{
	// this class will use only about calculate axis
	class first_case_axis{
//use equation is SumOfF is F{thruster} = ma{target} - F{gravity} - F{drag of liquid} - F{nature}
//										= ma         - mg         - k(v{current})^2   - F{nature}
		private:
			double force_gravity;
			double force_thruster;

		protected:
			double constant; // constant is k
			double force_constant; //force_constant is F{nature}

		
		public:
			first_case_axis(double gravity, double force_constant);
			float calculate_force_axis(double target_acceleration , double current_velocity);
			void set_constant(double constant, double force_constant);	
	}
// set value will constant always forr calculate
	first_case_axis::first_case_axis(float gravity , float force_constant){
		std::cout 	<< "Init first case about find force \n";
		std::cout 	<< "Input value for gravity is " << gravity
					<< " and value for force_constant " << force_constant << "\n";
		this->force_gravity = gravity*mass;
		this->force_constant = force_constant;
	}

// We will use this equation
//use equation is SumOfF is F{thruster} = ma{target} - F{gravity} - F{drag of liquid} - F{nature}
	double first_case_axis::calculate_force_axis(	double target_acceleration, 
													double current_velocity){
		force_thruster =	mass * target_acceleration 
						-	this->force_gravity
						-	this->constant * current_velocity * current_velocity
						-	this->force_constant;
	}

	void first_coase_axis::set_constant(double constant, double force_constant){
		this->constant = constant;
		this->force_constant = force_constant;
	}
}
