// please :set nu tabstop=4

// initial code run 1 time
void init(){
	PID_position = ( find_velocity::second_case)calloc 
						(6 , sizeof( find_velocity::second_case));
	PID_velocity = ( find_velocity::second_case)calloc
						(6 , sizeof( find_velocity::second_case));
	for(int count = 0 ; count < 6 count++){
		PID_position[count].set_constant(0 ,0 ,0);
		PID_velocity[count].set_constant(0 ,0 ,0);
	}
}
