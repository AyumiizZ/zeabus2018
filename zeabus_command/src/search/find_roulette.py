#!/usr/bin/python2

from control_auv		import auv_control
from zeabus_vision.srv	import *

class find_roulette:

	def __init__( self , world_yaw_slot , world_distance_slot , limit_distance , move_distance):

#------------------ setup handle for use control ------------------------------------
		self.auv = auv_control( "find a roulette")

#------------------ setup service for call data from vision -------------------------
		
#------------------------------------------------------------------------------------
		self.checkpoint = [0 , 0 , 0 , 0 , 0 , 0]
		
		temporary = self.auv.find_target( "xy")
		self.checkpoint[0] = temporary[0]
		self.checkpoint[1] = temporary[1]
		temporary = self.auv.find_target( "yaw")
		self.checkpoint[5] = temporary[5]
		
		self.mode = 1;
		self.time = 0.2;
		self.count = 0;		
		self.auv.relative_yaw( world_yaw_slot )

		self.limit = limit_distance
		self.move_y = move_distance
		self.direction = 1
		self.move_x = 0.5
		self.count_move = 0
		self.agree_center = 0.1
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
				if self.auv.check_position( "yaw" , 0.02).ok :
					if self.count > 4 :
						print "go to mode 2"
						self.mode = 2
						self.count = 0
						self.auv.relative_xy( world_distance_slot )
						self.auv.absolute_depth( -1.0 )
					else :
						print "In mode 1 count is " + str( self.count )
						self.count += 1
				else:
					print "In mode 1 reset count"
					self.count = 0 

			elif ( self.mode == 2 ): # this mode prepare to find
				if (self.auv.check_position( "xy" , 0.03).ok and 
					self.auv.check_position( "yaw" , 0.02).ok and
					self.auv.check_position( "z" , 0.1).ok ):
					if self.count > 4 :
						print "go to mode 3"
						self.mode = 3
					else :
						print "In mode 2 count is " + str( self.count )
						self.count += 1
				else:
					print "In mode 2 reset count"
					self.count = 0

			elif(self.mode == 3 ): # this mode forward survey
				if self.auv.check_position( "xy" , 0.03).ok :
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
				if self.result_of_roulette() : 
					self.past_mode = 3
					self.mode = 5
					self.count = 0
#------------------------------------------ end -------------------------------------------------

			elif(self.mode == 4 ): # this mode backward survey
				if self.auv.check_position( "xy" , 0.03).ok	:
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
				if self.result_of_roulette() : 
					self.past_mode = 4
					self.mode = 5
					self.count = 0
#------------------------------------------ end -------------------------------------------------

			elif(self.mode == 5 ): # for find a roulette
				self.found , self.roulette_x , self.roulette_y = target_for_roulette()
				if( self.found ) :
					if( self.roulette_x < self.agree_center 
							and self.roulette_x > -1 * self.agree_center ):	
						self.x_ok = True
						self.x_direction = 0
					elif (self.x_direction == 0 
							or self.x_direction == self.find_direction( self.roulette_x )):
						self.auv.relative_xy(self.roulette_x , 0)
						self.x_reverse = -1 * self.roulette_x / 2
					else:
						self.x_direction = self.find_direction( self.x_reverse )
						self.auv.relative_xy( self.x_reverse , 0)
						self.x_reverse = -1 * self.x_reverse / 2

					if( self.roulette_y < self.agree_center 
							and self.roulette_y > -1 * self.agree_center ):	
						self.y_ok = True
						self.y_direction = 0
					elif (self.y_direction == 0 
							or self.y_direction == self.find_direction( self.roulette_y )):
						self.auv.relative_xy( 0 , self.roulette_y )
						self.y_reverse = -1 * self.roulette_y / 2
					else:
						self.y_direction = self.find_direction( self.y_reverse )
						self.auv.relative_xy( self.y_reverse , 0)
						self.y_reverse = -1 * self.y_reverse / 2

				else:
					self.mode = self.past_mode
					self.x_direction = 0
					self.y_direction = 0

				temporary_z = self.auv.find_target("z")

				if( temporary_z > -1.1 ):
					print "now target depth is -2.1"
					self.auv.absolute_depth( -2.1 )
					self.found , self.roulette_x , self.roulette_y = target_for_roulette()
						

			rospy.sleep( self.time )

	def result_of_roulette(self):
		# service for call vision for find roulette	
#		result =

		return True

	def target_for_roulette(self):
		# service for call vision for move to above roulette

		return True , 0 , 0

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
	find_roulette = find_roulette( 0 , 5 , 5 , 0.5)
	find_roulette.main()
