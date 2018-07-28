How to run control in machine (4 step) in Machine
---minimal
---localization_noswtich
---rosrun zeabus_controller new_thrust_mapper_2018.py
---rosrun zeabus_controller new_2018_controller

This controll use x y z roll pitch yaw to calculate value and use matrix transform to convert to pwn for moter
and use PID to calcuater about that
										Supasan Komonlit, 2018

Now we will decription about calculate force for target velocity
--- In the past we use formular is force = K * velocity * velocity
--- now we use formulat is force = K * velocity
--- that mean if We try to calculate about min force and max fore in Y and X axies 
--- we will find in X axies : MIN 0.12 MAX is 3.5
--- so in Y axies have use thruster same X aies that mean : MIN 0.12 MAX is 3.5
		form equation one variable

			0.12 = K * 0.01 + C  ----1
			3.5  = K * 1 + C     ----2
	
			3.38 = 0.99K
			K = 3.41414141
			C = 0.08586
