------------------------------------------
---------------- Message -----------------
----------	Supasan	  Komonlit	----------
--Now we have to control 2 type

--->first type:
------ USE 2 file
--------> new_controller_2018.cpp in src
--------> 2018_thruster_mapper_origin.py
			in script
		this use PID to control output
		PID from error of orientation
----------->status : already<-------------


--->second type
------ USE 2 file
--------> force_controller.cpp in src
--------> 2018_thruster_mapper_force.py
			in script
		this use PID to find_velocity
			from error position but 
			whave way to change
		and get error velocity to find
			accelaration
		and bring accelaration to find
			force for send to thruster
---------->status : can't use<------------
---------------now progress---------------
------		test file calculate		------	
------------------------------------------
