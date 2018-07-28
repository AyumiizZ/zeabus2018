#!/usr/bin/python2

import rospy
from std_msgs.msg       import String
from control_auv	import auv_control
from zeabus_vision.srv	import vision_srv_roulette
from zeabus_vision.msg	import vision_roulette

class find_roulette:

	def __init__( self , world_yaw_slot , world_distance_x , world_distance_y , 
                            limit_distance , move_distance , start_depth):

#------------------ setup handle for use control ------------------------------------
		self.auv = auv_control( "find a roulette")

#------------------ setup service for call data from vision -------------------------
		print "wait service /vision_roulette"
		self.information 	= rospy.ServiceProxy('vision_roulette' , vision_srv_roulette)
		print "have service /vision_roulette"
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
				result , move_x , move_y = self.result_of_roulette()
				if  result :
					self.auv.relative_xy( move_x , move_y ) 
					temporary = self.auv.find_target( "xy")
					self.checkpoint[0] = temporary[0]
					self.checkpoint[1] = temporary[1]
					self.past_mode = 3
					self.mode = 5
					self.count = 0
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
				result , move_x , move_y = self.result_of_roulette()
				if  result :
					self.auv.relative_xy( move_x , move_y ) 
					temporary = self.auv.find_target( "xy")
					self.checkpoint[0] = temporary[0]
					self.checkpoint[1] = temporary[1]
					self.past_mode = 4
					self.mode = 5
					self.count = 0
#------------------------------------------ end -------------------------------------------------

			elif( self.mode == 5 ):
				print( "Now mode 05" )
				if not self.auv.check_position( "xy", 0.05):
					print("Waiting for constant position")
				else:	
					self.move_to_roulette()

			elif( self.mode == 7):
				print( "finish move" )
				self.auv.relative_depth( -1 )
				break
######################################### old code ##############################################
#			elif(self.mode == 5 ): # for when find a roulette
#				print "Now mode 05"
#				self.found , self.roulette_x , self.roulette_y = self.target_for_roulette()
#				if( self.found ) :
#					if( self.roulette_x < self.agree_center 
#							and self.roulette_x > -1 * self.agree_center ):	
#						self.x_ok = True
#						self.x_direction = 0
#					elif (self.x_direction == 0 
#							or self.x_direction == self.find_direction( self.roulette_x )):
#						self.auv.relative_xy(self.roulette_x , 0)
#						self.x_reverse = -1 * self.roulette_x / 2
#						self.x_ok = False
#					else:
#						self.x_direction = self.find_direction( self.x_reverse )
#						self.x_ok = False
#						self.auv.relative_xy( self.x_reverse , 0)
#						self.x_reverse = -1 * self.x_reverse / 2
#
#					if( self.roulette_y < self.agree_center 
#							and self.roulette_y > -1 * self.agree_center ):	
#						self.y_ok = True
#						self.y_direction = 0
#					elif (self.y_direction == 0 
#							or self.y_direction == self.find_direction( self.roulette_y )):
#						self.auv.relative_xy( 0 , self.roulette_y )
#						self.y_ok = False
#						self.y_reverse = -1 * self.roulette_y / 2
#					else:
#						self.y_direction = self.find_direction( self.y_reverse )
#						self.y_ok = False
#						self.auv.relative_xy( self.y_reverse , 0)
#						self.y_reverse = -1 * self.y_reverse / 2
#
#				else:
#					self.mode = self.past_mode
#					self.x_direction = 0
#					self.y_direction = 0
#
#				temporary_z = self.auv.find_target("z")
#
#				if( temporary_z > -1.1 and self.found and self.x_ok and self.y_ok):
#					print "now target depth is -2.1"
#					self.auv.absolute_depth( -3.1 )
#                                       break
#                                        
#				if( temporary_z < -1.9 and self.found and self.x_ok and self.y_ok):
#					print "finish move "
#					self.auv.absolute_depth( -3.5 )
#					break
#################################################################################################

			rospy.sleep( self.time )

	def move_to_roulette( self ):
		count_found = 0
		count_false = 0
		now_center = False
		x_ok = False
		y_ok = False
		print "---------------------function of move to roulette------------------------------"
		while( not rospy.is_shutdown() ):
			result = self.information( String("roulette") , String("find"))
			if( result.data.appear ):
				print( "data appear and "
						+ " cx for move is " + str(result.data.cy) 
						+ " : cy for move is " + str( -1 * result.data.cx) )
				count_false = 0
				if( self.center_or_not( result.data.cy ) ):
					velocity_x = 0
					x_ok = True
					print("OK move X")
				else:		
					x_ok = False		
					velocity_x = result.data.cy * 0.5
					print "move x is " + velocity_x
				if( self.center_or_not( result.data.cx ) ):
					velocity_y = 0
					y_ok = True
					print("OK move y")
				else:
					y_ok = True
					velocity_y = result.data.cx * -1 * 0.5
					print "move y is " + velocity_y
				
				if( x_ok and y_ok ):
					now_center = True
					self.auv.send_velocity( [0 , 0 , 0 , 0 , 0 , 0])
					self.auv.relative_depth("-0.3")
				else:
					now_center = False
					self.auv.send_velocity( [ velocity_x , velocity_y , 0 , 0 , 0 , 0] )
			else:
				if(now_center):
					self.mode =7
					break
				else:
					if( count_false > 4):
						print( "False in move return check point")
						self.auv.absolute_xy( self.check_position[0] , self.check_position[1] )
						self.mode = self.past_mode
					else:
						count_false += 1
						print("print count_false is " + str(count_false))
			rospy.sleep(0.1)

	def result_of_roulette(self):
		# service for call vision for find roulette	
		# how to use --> result = self.information( tast , request)
		# result must return cx xy area appear
		print "------------------------ find roulette -----------------------------"
		count_found = 0 
		count_false = 0
		move_x = 0
		move_y = 0
 		while( count_found < 10 and count_false < 10 and not rospy.is_shutdown() ):
			print "count_found : " + str(count_found) + " count_false : " + str(count_false)
			if not self.auv.check_position( "xy" , 0.06) :
				rospy.sleep( self.time)
				print "false position"
				continue
			result = self.information( String("roulette") , String("find") )
			if( result.data.appear ):
				move_x *= count_found
				move_y *= count_found
				count_found += 1
				move_x = ( move_x + (  1 * result.data.cy ) ) / count_found
				move_y = ( move_y + ( -1 * result.data.cx ) ) / count_found
				print( "Round " + str(count_found) + " ( move_x,move_y ) : " + str(move_x) + ","
						+ str(move_y) )
			else:
				count_false += 1
				print( "False " + str(count_false) )
		if( count_found == 10 ):
			print( "see all roulette at " + str(move_x) + " " + str(move_y) )
			return True , move_x , move_y
		else:
			print( "Don't see all roulette")
			return False , 0 , 0 		


	def target_for_roulette(self):
		# service for call vision for move to above roulette
		print "------------------------ find roulette -----------------------------"
		count_found = 0 
		count_false = 0
		move_x = 0
		move_y = 0
		area = 0
 		while( count_found < 10 and count_false < 10 and not rospy.is_shutdown() ):
			print "count_found : " + str(count_found) + " count_false : " + str(count_false)
			if not self.auv.check_position( "xy" , 0.06) :
				rospy.sleep( self.time)
				print "false position"
				continue
			result = self.information( String("roulette") , String("find") )
			if( result.data.appear ):
				move_x *= count_found
				move_y *= count_found
				count_found += 1
				move_x = ( move_x + (  1 * result.data.cy )) / count_found
				move_y = ( move_y + ( -1 * result.data.cx )) / count_found
				print( "Round " + str(count_found) + " ( move_x,move_y ) : " + str(move_x) + ","
						+ str(move_y) )
			else:
				count_false += 1
				print( "False " + str(count_false) )
		if( count_found == 10 ):
			print( "see target roulette at " + str(move_x) + " " + str(move_y) )
			return True , move_x , move_y
		else:
			print( "Don't see target roulette")
			return False , 0 , 0 		


	def center_or_not( self , problem):
		if( problem < self.agree_center and problem > -1 * self.agree_center): 
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

if __name__=='__main__':
 # world_yaw_slot , world_distance_x , world_distance_y , limit_distance , move_distance , depth

# first argument please enter absolute yaw of robot
# second arguments
	rospy.init_node("find_roulette")
	find_roulette = find_roulette( -1.57 , 2 , 3 , 5 , 0.5 , -1)
#	find_roulette = find_roulette( -1.57 , 0 , 0 , 5 , 0.6 , -1)
	find_roulette.main()
	print("finish found")
