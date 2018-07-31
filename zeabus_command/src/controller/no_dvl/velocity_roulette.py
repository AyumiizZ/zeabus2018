#!/usr/bin/python2

import rospy , math
from std_msgs.msg       import String
from control_auv		import auv_control
from zeabus_vision.srv	import vision_srv_roulette
from zeabus_vision.msg	import vision_roulette

class go_roulette:

	def __init__( self , absolute_yaw , relative_x , relative_y , start_depth ):

#-------------------------------------- about survey ------------------------------------------
		self.move = 0.5
		self.limit_move = 2
		self.time = rospy.Rate( 30 )

#------------------------------------ for order to auv ------------------------------------
		self.auv = auv_control("go_roulette") 
		
#------------------ setup service for call data from vision -------------------------
		print "wait service /vision_roulette"
		self.information 	= rospy.ServiceProxy('vision_roulette' , vision_srv_roulette)
		print "have service /vision_roulette"
#-----------------------------------------------------------------------------------------

		self.checkpoint = [ 0 , 0 , 0 , 0 , 0 , 0]

# -------------------------------- combo to check target point ----------------------------
		temporary = self.auv.find_target( "xy")
		self.checkpoint[0] = temporary[0]
		self.checkpoint[1] = temporary[1]
		temporary = self.auv.find_target( "yaw")
		self.checkpoint[5] = temporary[0]

		self.start_yaw = absolute_yaw
		self.start_x = relative_x
		self.start_y = relative_y
		self.start_depth = start_depth

	def main( self , limit_time ):
				 
		self.limit_time = limit_time
		self.start_time = rospy.Time()
		self.now_time = rospy.Time()

		self.auv.absolute_yaw( self.start_yaw )
		self.count = 0
		while(not self.auv.check_position( "yaw" , 0.1 ) and self.count < 60):
			print( "Waiting for absolute yaw : " + str( self.count ) )
			self.count += 1
			self.time.sleep()

		self.auv.absolute_depth( self.start_depth )
		self.count = 0
		while( not self.auv.check_position( "z" , 0.1) and self.count < 60):
			print( "Waiting for absolute depth : " + str( self.count ))
			self.count += 1
			self.time.sleep()

		self.auv.relative_xy( self.start_move_x , self.start_move_y)
		self.count = 0
		while( not  self.auv.check_position( "xy" , 0.1) and self.count < 240):
			print( "Waiting for relative xy : " + str( self.count ))
			self.count += 1
			self.time.sleep()
		
		self.survey_forward()

	def survey_forward( self ):
		self.now_time = rospy.Time()

		print( "now use time is " + str( (self.now_time - self.start_time ).to_sec()))		

		self.count_move = 0
		
		mode = 0
		
		while( self.count_move < self.limit_move and not rospy.is_shutdown() ):
			found , cx , cy = self.result_information()
			print( "-------------------------------------------------------------------------")
			print( "summary result is " + str( found) + " so cx : " + str( cx) + " cy : " + 
					str(cy))
			if( not found ):
				self.count = 0
#				self.auv.relative_xy( self.move , 0)
				self.auv.continue_move( [0.2 , 0 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
				while( not self.auv.check_position( "xy" , 0.1) and self.count_move < 60):
					print( "Waiting for move xy : " + str( self.count))
					self.count_move += 1
					self.time.sleep()
			else:
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
				break

		self.count = 0		
		if( not found ):
			print( "Can't find in forward mode next backword")
			self.auv.continue_move( [ 0 , -0.2 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
			while( not self.auv.check_position( "xy" , 0.1) and self.count < 60):
				print( "Waiting for move xy : " + str(self.count) )
				self.count_move += 1
				self.time.sleep()
			self.survey_backward()
		else:
			print( "I found object")
			self.move_on_target()

	def	survey_backward( self ):
		self.now_time = rospy.Time()
		
		print( "now use time is " + str( (self.now_time - self.start_time ).to_sec()))

		self.count_move = 0 

		mode = 0

		while( self.count_move < self.limit_move and not rospy.is_shutdown() ):
			found , cx , cy = self.result_information()
			print( "-------------------------------------------------------------------------")
			print( "summary result is " + str( found ) + " so cx : " + str( cx) + " cy : " + 
					str(cy))
			if( not found ):
				self.count = 0
#				self.auv.relative_xy( self.move , 0)
				self.auv.continue_move( [-0.2 , 0 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
				while( not self.auv.check_position( "xy" , 0.1) and self.count_move < 60):
					print( "Waiting for move xy : " + str( self.count))
					self.count_move += 1
					self.time.sleep()
			else:
				print( "I found that object")
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
				break

		self.count = 0		
		if( not found ):
			print( "Can't find in backword mode next forward")
			self.auv.continue_move( [ 0 , -0.2 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
			while( not self.auv.check_position( "xy" , 0.1) and self.count < 60):
				print( "Waiting for move xy : " + str(self.count) )
				self.count_move += 1
				self.time.sleep()
			self.survey_forward()
		else:
			print( "I found object")
			self.move_on_target()
			while( not rospy.is_shutdown() ):
				found , cx , cy = self.result_information()
				print( "--------------------------------------------------------------------" )

	def	move_on_target( self ):
		ok_x = False
		ok_y = False
		while( not rospy.is_shutdown() ):
			found , cx , cy = self.result_information()
			print( "-------------------------------------------------------------------------")
			print( "summary result is " + str( found ) + " so cx : " + str( cx) + " cy : " + 
					str(cy))
			if( not found ):
				print( "Now I'm center of roulette")
			else:
				print( " I found that ")
				ok_x = agree_that( 0 , cy , 0.1)
				ok_y = agree_that( 0 , cx , 0.1)
				print( "result of center is " + str(ok_x) + " and y : " + str(ok_y) )
				if( not ( ok_x or ok_y)):
					self.auv.send_velocity([ cy * 0.6 , cx * -0.8 , 0 , 0 , 0 , 0 ])
				elif not ok_x:
					self.auv.send_velocity([ cy * 0.6 , 0 , 0 , 0 , 0 , 0 ])
				elif not ok_y:
					self.auv.send_velocity([ 0 , cx * -0.8 , 0 , 0 , 0 , 0 ])
				else:
					self.auv.send_velocity([ 0 , 0 , -0.2 , 0 , 0 , 0 ])
			self.time.sleep()


	def result_information( self ):		
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
			return True , move_y , move_x
		else:
			print( "Don't see all roulette")
			return False , 0 , 0 		

	def agree_that( self , current , target , agree ):
		diff = target - current
		if( -1 * agree < target and target < agree):
			return True
		else:
			return False

if __name__=='__main__':

	# please input absolute_yaw , relative_x , relative_y , start depth
	find_roulette = go_roulette( 0 , 0 , 0 , -1)
