#include 	<ros/ros.h> // include if you wan to use time
#include 	<math.h> // use math for calculate

// this code will use to calculate what force you want in each axis
// the equation use is cm(n-1) /  v^n = distance when force from thruster is 0
// we have decide n = 2 by Picky Pack
// and we have to decide about velocity = cv^n	
// and so n = 2 by Picky Pack

const static double mass;


// normal from is F(thruster) F(gravity) F(drag of liquid)
class force_axis{
	public: // this type of access will allow every way to access that
		double	constant_value; // this is c
		double	axis_gravity; // this is g
		double	force_thruster; // this is will answer for force in each axis
		double	
}
