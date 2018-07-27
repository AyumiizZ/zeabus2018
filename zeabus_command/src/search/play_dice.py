#!/usr/bin/python2

import rospy , math
from std_msgs.msg       import String
from control_auv		import auv_control
from zeabus_vision.srv	import vision_srv_shoot_craps
from zeabus_vision.msg	import vision_shoot_craps

class play_dice():

	def __init__(self , start_depth , hit_area , distance ):

#------------------ setup handle for use control ------------------------------------
		self.auv = auv_control( "find a cash_in_your_chip")

#------------------ setup service for call data from vision -------------------------
		print "wait service /vision_shoot_craps"
		self.information 	= rospy.ServiceProxy('vision_shoot_craps' , vision_srv_shoot_craps)
		print "have service /vision_shoot_craps"
#------------------------------------------------------------------------------------
		self.checkpoint = [0 , 0 , 0 , 0 , 0 , 0]
		self.auv.absolute_depth( start_depth )
		self.auv.relative_xy( distance , 0 )
		temporary = self.auv.find_target( "xy")
		self.checkpoint[0] = temporary[0]
		self.checkpoint[1] = temporary[1]
		temporary = self.auv.find_target( "z" )
		self.checkpoint[2] = temporary[0]
		temporary = self.auv.find_target( "yaw")
		self.checkpoint[5] = temporary[0]

		self.finish_5_6 = [ False , False]

		self.direction_z = 0 # 0 for go down 1 for go up
		self.mode = 0
		self.time = 0.1

		self.last_found = False
		self.last_area = 0
		self.last_cx = 0
		self.last_cy = 0
                self.last_velocity = [ 0 , 0 , 0 , 0 , 0 , 0]

	def	main(self):
		while( not rospy.is_shutdown() ):
			if( self.mode == 0): # for move to find check point and play roulette
				move_x , move_y , move_z = self.find_all_dice()
				temporary = self.auv.find_target( "xy")
				self.checkpoint[0] = temporary[0]
				self.checkpoint[1] = temporary[1]
				temporary = self.auv.find_target( "z" )
				self.checkpoint[2] = temporary[0]
				temporary = self.auv.find_target( "yaw")
				self.checkpoint[5] = temporary[0]		
				self.auv.relative_xy( move_x , move_y)
				self.auv.relative_depth( move_z)

			elif( self.mode == 2): # for play 6
				move_x , move_y , move_z , area ,found = self.find_dice(6)
				if( found ):
					self.last_found = True
					self.last_area = area
#					self.auv.send_velocity( [0 , move_y , move_z 
 #                                                               , 0     ,   0    ,   0])
					self.auv.relative_xy( move_x , move_y)
                                        self.auv.relative_depth( move_z)
                                        rospy.sleep( self.time)
					for i in range (40):
                                                print("waiting")
						if( self.auv.check_position("xy" , 0.04)):
							break
						rospy.sleep( self.time)
					self.last_velocity = [ 0 , -1.3 * move_y 
                                                                ,0    , 0     
                                                                ,   0    ,   0]
				else:
					if( self.last_area > 0.65 ):
						print("last move")
#						move = input("Distance for last move")
#						self.auv.relative_xy( move , 0)
						break
                                        else:
						print( "reverse move")
                                                print( "move y is " + str(self.last_velocity[1])
                                                    + "move z is " + str(self.last_velocity[2]))
						self.auv.send_velocity( self.last_velocity )

			rospy.sleep( self.time )	

	def find_dice( self , number):
		print("----------------- play " + str(number) + " ---------------------------")
		if( number == 5):
			num = 1
		else:
			num = 2
		move_y = 0
		move_z = 0
		area = 0
		count_false = 0
		count_true = 0
		while( not rospy.is_shutdown() and count_true < 2 and count_false < 2):
#			if( self.auv.check_position("xy" , 0.05) and self.auv.check_position("yaw" , 0.2)
#						and self.auv.check_position("z" , 0.1)):
                        if( self.auv.check_position("yaw" , 0.1) ):
				result = self.information( String('shoot_craps') )
				result = result.data
				self.print_result( result )
				if( result.appear[num] ):
					count_true += 1 
					move_y += ( result.cx[num] * -1 )
					move_z += result.cy[num]
					area += result.area[num]
				else:
					count_false += 1
		if( count_true == 2):
			return 0.3  , ( move_y/2 * 0.5 ) , -1 * ( move_z/2 * 0.5) , area/2 , True
		else:
			return 0 , 0 , 0 , 0 , False

	def find_all_dice(self):
		print("-------------------- find all dice ------------------------")
		while( not rospy.is_shutdown() ):
			if( self.auv.check_position("xy" , 0.05) and self.auv.check_position("z" , 0.1) ):
				result = self.information( String('shoot_craps') )
				result = result.data
				self.print_result( result )
				count = 0
				if( result.appear[1]):
					count += 1
				if( result.appear[2]):
					count += 1
				if( count == 2 ):
					middle_y = ( result.cy[1] + result.cy[2] ) / 2
					middle_x = ( result.cx[1] + result.cx[2] ) / 2
					print("find 5 and 6 move y : " + middle_x + "\tmove z : " + middle_y)
					return 0 , middle_x , middle_y* -1
				elif( count == 1):
					if( result.appear[1] ):
						print(" find 5 ")
						self.mode = 1 # play 5
						return 0 , 0 , result.cy[1]
					if( result.appear[2] ):
						print(" find 6 ")
						self.mode = 2 # play 6
						return 0 , 0 , result.cy[2]
				else:	
					return -0.3 , 0 , 0
			else:
				print("Waiting position")
			
	def print_result(self , result):
		for i in range(len(result.appear)):
			print("for dice 2 cx : " + str(result.cx[0]) + "\tcy : " + str(result.cy[0]) + "\tarea : " 
									+ str(result.area[0]) )
			print("for dice 5 cx : " + str(result.cx[1]) + "\tcy : " + str(result.cy[1]) + "\tarea : " 
									+ str(result.area[1]) )
			print("for dice 6 cx : " + str(result.cx[2]) + "\tcy : " + str(result.cy[2]) + "\tarea : " 
									+ str(result.area[2]) )


if __name__=='__main__':
	rospy.init_node('play_dice')
	play_dice = play_dice( -3 , 0.4 , 0)
	play_dice.main()
	print("finish play dice")
