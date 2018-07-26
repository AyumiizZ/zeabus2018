#!/usr/bin/python2

import rospy , math
from std_msgs.msg       import String
from control_auv	import auv_control
from zeabus_vision.srv	import vision_srv_cash_in
from zeabus_vision.msg	import vision_cash_in

class find_cash_in_your_chip:

	def __init__( self , world_yaw_slot , world_distance_x , world_distance_y , 
                            limit_distance , move_distance , start_depth):

#------------------ setup handle for use control ------------------------------------
		self.auv = auv_control( "find a cash_in_your_chip")

#------------------ setup service for call data from vision -------------------------
		print "wait service /vision_cash_in_your_chip"
		self.information 	= rospy.ServiceProxy('vision_cash_in' , vision_srv_cash_in)
		print "have service /vision_cash_in_your_chip"
#------------------------------------------------------------------------------------
		self.checkpoint 	= [0 , 0 , 0 , 0 , 0 , 0]
		
		temporary = self.auv.find_target( "xy")
		self.checkpoint[0] = temporary[0]
		self.checkpoint[1] = temporary[1]
		temporary = self.auv.find_target( "yaw")
		self.checkpoint[5] = temporary[0]
                self.distance_x = world_distance_x
                self.distance_y = world_distance_y

		self.mode = 1;
		self.time = 0.2;
		self.count = 0;		
		self.auv.absolute_yaw( world_yaw_slot )
		self.start_depth = start_depth

		self.limit = limit_distance
		self.move_y = -1*move_distance
		self.direction = 1
		self.move_x = move_distance
		self.count_move = 0
		self.agree_center = 0.01
		# when found roulette
		self.record_point = [ 0 , 0]
		self.x_ok = False
		self.y_ok = False
		self.x_reverse = 0
		self.y_reverse = 0
		self.x_direction = 0
		self.y_direction = 0
		self.past_mode = 0

	def main(self):
		while( not rospy.is_shutdown() ):
			if(self.mode == 1): # this mode use for assign yaw
				print "Now mode 01"
				if self.auv.check_position( "yaw" , 0.1) :
					if self.count > 3 :
						print "go to mode 2"
						self.mode = 2
						self.count = 0
						self.auv.relative_xy( self.distance_x 
                                                            , self.distance_y )
						self.auv.absolute_depth( self.start_depth )
					else :
						print "In mode 1 count is " + str( self.count )
						self.count += 1
				else:
					print "In mode 1 reset count"
					self.count = 0 

			elif ( self.mode == 2 ): # this mode prepare to find
				print "Now mode 02"
				if (self.auv.check_position( "xy" , 0.03) and 
					self.auv.check_position( "yaw" , 0.02) and
					self.auv.check_position( "z" , 0.1) ):
					if self.count > 1 :
						print "go to mode 3"
						self.mode = 3
					else :
						print "In mode 2 count is " + str( self.count )
						self.count += 1
				else:
					print "In mode 2 reset count"
					self.count = 0

			elif(self.mode == 3 ): # this mode forward survey
				print "Now mode 03"
				if self.auv.check_position( "xy" , 0.05) :
					if self.count > 4 :
						self.count_move += self.move_x
						if self.count_move > self.limit :
							print "move go to mode 4"
							self.auv.relative_xy( 0 , self.move_y)
							self.mode = 4 
							self.count_move = 0
						else :
							print "move in mode 3"
							self.auv.relative_xy( self.move_x , 0)
					else :
						print "in mode 3 search again"
						self.count += 1
#------------------------------------- for search roulett ---------------------------------------
				result , move_x , move_y = self.result_of_cash_in_your_chip()
				if  result != 0 :
					print("finish and found result is " + str(result))
					self.auv.relative_xy( move_x , move_y )
					temporary = self.auv.find_target( "xy")
					self.checkpoint[0] = temporary[0]
					self.checkpoint[1] = temporary[1]
					self.past_mode = 4
					self.mode = 5
					self.count = 0
					break
                                else:
					print("not found")
#------------------------------------------ end -------------------------------------------------

			elif(self.mode == 4 ): # this mode backward survey
				print "Now mode 04"
				if self.auv.check_position( "xy" , 0.05)	:
					if self.count > 4 :
						self.count_move -= self.move_x
						if self.count_move < -1 * self.limit :
							print "move go to mode 3"
							self.auv.relative_xy( 0 , self.move_y)
							self.mode = 3 
							self.count_move = 0
						else :
							print "move in mode 4"
							self.auv.relative_xy( -1 * self.move_x , 0)
					else :
						print "in mode 4 search again"
						self.count += 1
#------------------------------------- for search roulett ---------------------------------------
				result , move_x , move_y = self.result_of_cash_in_your_chip()
				if  result != 0 :
					print("finish and found result is " + str(result))
					self.auv.relative_xy( move_x , move_y )
					temporary = self.auv.find_target( "xy")
					self.checkpoint[0] = temporary[0]
					self.checkpoint[1] = temporary[1]
					self.past_mode = 4
					self.mode = 5
					self.count = 0
					break
                                else:
					print("not found")
#------------------------------------------ end ------------------------------------------------
			elif( self.mode == 5 ):
				print( "Now mode 05" )
				if not self.auv.check_position( "xy", 0.05):
					print("Waiting for constant position")
#				else:
#					self.move_to_cash_in_your_chip():

			elif( self.mode == 7):
				print( "finish move" )
				self.auv.relative_depth( -1 )
				break

			rospy.sleep( self.time )

#	def move_to_cash_in_your_chip( self ):
#       pass

	def result_of_cash_in_your_chip(self):
		# service for call vision for find cash_in_your_chip	
		# how to use --> result = self.information( tast , request)
		# result must return cx1 cy1 cx2 cy2 area mode
		print "------------------------ find cash_in_your_chip -----------------------------"
		count_0 = 0
		count_1 = 0
		count_2 = 0
		found_cx = 0
		found_cy = 0
		double_found_cx_01 = 0
		double_found_cy_01 = 0	
		double_found_cx_02 = 0
		double_found_cy_02 = 0	
		while( count_0 < 10 and count_1 < 10 and count_2 < 10 ):
			result = self.information( String("cash_in") , String("bin") )
			if result.data.mode == 0 :
				count_0 += 1
			elif result.data.mode == 1 :
				found_cx *= count_1
				found_cy *= count_1
				count_1 += 1
				found_cx = ( found_cx + result.data.cx1 ) / count_1 
				found_cy = ( found_cy + result.data.cy1 ) / count_1
				
#			elif result.data.mode == 2 :
			else:
				match_type = self.check_match_type( [double_found_cx_01 , double_found_cy_01] ,
													[double_found_cx_02 , double_found_cy_02] ,
													[result.data.cx1	, result.data.cy1]	,
													[result.data.cx2	, result.data.cy2])
				double_found_cx_01 *= count_2
				double_found_cx_02 *= count_2
				double_found_cy_01 *= count_2
				double_found_cy_02 *= count_2
				count_2 += 1
				if match_type == 1 :
					double_found_cx_01 = ( double_found_cx_01 + double_found_cx_01 ) / count_2
					double_found_cx_02 = ( double_found_cx_02 + double_found_cx_02 ) / count_2
					double_found_cy_01 = ( double_found_cy_01 + double_found_cy_01 ) / count_2
					double_found_cy_02 = ( double_found_cy_02 + double_found_cy_02 ) / count_2
				else:
					double_found_cx_01 = ( double_found_cx_01 + double_found_cx_02 ) / count_2
					double_found_cx_02 = ( double_found_cx_02 + double_found_cx_01 ) / count_2
					double_found_cy_01 = ( double_found_cy_01 + double_found_cy_02 ) / count_2
					double_found_cy_02 = ( double_found_cy_02 + double_found_cy_01 ) / count_2
                print("after request data count_0 : " + str(count_0)
                    ,"                   count_1 : " + str(count_1)
                    ,"                   count_2 : " + str(count_2))
						
		if( count_0 == 10):
			return 0 , 0 , 0
		elif( count_1 == 10):
			move_x = found_cy 
			move_y = found_cx * -1
			return 1 , move_x , move_y
		elif( count_2 == 10): 
			move_x = ( double_found_cx_01 + double_found_cx_02 ) / 2
			move_y = ( double_found_cy_01 + double_found_cy_02 ) / -2 
			return 2 , move_x , move_y

	def center_or_not( self , problem):
		if( problem < self.agree_center and problem > -1 * self.agree_center ): 
			return True
		else:
			return False

	def find_direction( self , problem):
		if( problem < 0 ): 
			return -1
		elif( problem > 0):
			return 1
		else:
			return 0
# we will by origin_1
	def check_match_type( self , origin_1 , origin_2 , new_1 , new_2):
		distance_1_1 = self.find_distance( origin_1 , new_1 )
		distance_1_2 = self.find_distance( origin_1 , new_2 )
		if( distance_1_1 < distance_1_2): 
			return 1
		else: 
			return 0
	
	def find_distance( self , problem_1 , problem_2 ):
		return math.sqrt( math.pow( problem_1[0] - problem_2[0] , 2 ) 
						+ math.pow( problem_1[1] - problem_2[1] , 2 ) )

if __name__=='__main__':
 # world_yaw_slot , world_distance_x , world_distance_y , limit_distance , move_distance , depth

# first argument please enter absolute yaw of robot
# second arguments
	rospy.init_node("find_cash_in_your_chip")
	find_cash_in_your_chip = find_cash_in_your_chip( -1.57 , 0 , 0 , 4 , 0.5 , -1)
#	find_roulette = find_roulette( -1.57 , 0 , 0 , 5 , 0.6 , -1)
	find_cash_in_your_chip.main()
	print("finish found")
