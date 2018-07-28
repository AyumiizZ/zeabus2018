#!/usr/bin/python2

import rospy , math
from std_msgs.msg       import String
from control_auv		import auv_control
from zeabus_vision.msg	import vision_cash_in
from zeabus_vision.srv	import vision_srv_cash_in

class go_cash_in_your_chip:

	def __init__( self , absolute_yaw , relative_x , relative_y , start_depth , ):

#-------------------------------------- about survey ------------------------------------------
		self.move = 0.5
		self.limit_move = 5

#------------------------------------ for order to auv ------------------------------------
		self.auv = auv_control("go_cash_in_your_chip") 
		
#------------------ setup service for call data from vision -------------------------
		print "wait service /vision_cash_in_your_chip"
		self.information 	= rospy.ServiceProxy('vision_cash_in' , vision_srv_cash_in)
		print "have service /vision_cash_in_your_chip"
#-----------------------------------------------------------------------------------------

		self.checkpoint = [ 0 , 0 , 0 , 0 , 0 , 0]

# -------------------------------- combo to check target point ----------------------------
		temporary = self.auv.find_target( "xy")
		self.checkpoint[0] = temporary[0]
		self.checkpoint[1] = temporary[1]
		temporary = self.auv.find_target( "yaw")
		self.checkpoint[5] = temporary[0]

		self.mode = 1 	

		self.start_time = rospy.Time()
		self.limit_time = None

		self.start_yaw = absolute_yaw
		self.start_move_x = relative_x
		self.start_move_y = relative_y
		self.start_depth = start_depth

		self.time = rospy.Rate( 30 )

		self.target_velocity = [ 0 , 0 , 0 , 0 , 0 , 0]

		self.ok_x = False
		self.ok_y = False

	def main(self , limit_time):
		
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
			num_found , cx , cy = self.result_information()
			print( "-------------------------------------------------------------------------")
			print( "summary result is " + str( num_found) + " so cx : " + str( cx) + " cy : " + 
					str(cy))
			if( num_found == 0 ):
				self.count = 0
#				self.auv.relative_xy( self.move , 0)
				self.auv.continue_move( [0.2 , 0 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
				while( not self.auv.check_position( "xy" , 0.1) and self.count_move < 60):
					print( "Waiting for move xy : " + str( self.count))
					self.count_move += 1
					self.time.sleep()
			elif( num_found == 1):
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
				break
			else:
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
				break

		self.count = 0		
		if( num_found == 0 ):
			print( "Can't find in forward mode next backword")
			self.auv.continue_move( [ 0 , -0.2 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
			while( not self.auv.check_position( "xy" , 0.1) and self.count < 60):
				print( "Waiting for move xy : " + str(self.count) )
				self.count_move += 1
				self.time.sleep()
			self.survey_backward()
		elif( num_found == 1):
			print( "Find one move to function \" to one object \"")
			self.to_one_object()
		else:
			print( "Find two move to function \" to two object \"")
			self.to_two_object()

	def	survey_backward( self ):
		self.now_time = rospy.Time()
		
		print( "now use time is " + str( (self.now_time - self.start_time ).to_sec()))

		self.count_move = 0 

		mode = 0

		while( self.count_move < self.limit_move and not rospy.is_shutdown() ):
			num_found , cx , cy = self.result_information()
			print( "-------------------------------------------------------------------------")
			print( "summary result is " + str( num_found) + " so cx : " + str( cx) + " cy : " + 
					str(cy))
			if( num_found == 0 ):
				self.count = 0
#				self.auv.relative_xy( self.move , 0)
				self.auv.continue_move( [-0.2 , 0 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
				while( not self.auv.check_position( "xy" , 0.1) and self.count_move < 60):
					print( "Waiting for move xy : " + str( self.count))
					self.count_move += 1
					self.time.sleep()
			elif( num_found == 1):
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
				break
			else:
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
				break

		self.count = 0		
		if( num_found == 0 ):
			print( "Can't find in forward mode next backword")
			self.auv.continue_move( [ 0 , -0.2 , 0 , 0 , 0 , 0 ] , 50 , 0.01 )
			while( not self.auv.check_position( "xy" , 0.1) and self.count < 60):
				print( "Waiting for move xy : " + str(self.count) )
				self.count_move += 1
				self.time.sleep()
			self.survey_forward()
		elif( num_found == 1):
			print( "Find one move to function \" to one object \"")
			self.to_one_object()
		else:
			print( "Find two move to function \" to two object \"")
			self.to_two_object()

	def to_one_object( self ):
		print("Welcome mode to center of one object")

		self.now_time = rospy.Time()
		
		print( "now use time is " + str( (self.now_time - self.start_time ).to_sec()))

		temporary = self.auv.find_target( "xy")
		self.checkpoint[0] = temporary[0]
		self.checkpoint[1] = temporary[1]
		temporary = self.auv.find_target( "yaw")
		self.checkpoint[5] = temporary[0]

		count_false = 0

		while( not rospy.is_shutdown() ):
			num_found , cx , cy = self.result_information()
			print( "--------------------------- in one object ---------------------------------")
			print( "summary result is " + str( num_found) + " so cx : " + str( cx) + " cy : " + 
					str(cy))
			if( num_found == 1 ):
				print("found one")
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
			elif( num_found == 2):
				print("now found 2")
				self.auv.send_velocity([ cy * 0.6 ,  cx * -1 * 0.8 , 0 , 0 , 0 , 0 , 0 , 0])
				break
			else:
				print( "not found anything")
				count_false += 1
				if( count_false == 5)
					self.auv.absolute_xy( self.checkpoint[0] , self.checkpoint[1])
					self.auv.absolute_yaw( self.checkpoint[5] )

	def to_two_object( self ):
		print( "Welcomde mode to center of two object")

		self.now_time = rospy.Time()
		
		print( "now use time is " + str( (self.now_time - self.start_time ).to_sec()))

		while( not rospy.is_shutdown() ):
			num_found , cx , cy = self.result_information()
			print( "--------------------------- in two object ---------------------------------")
			print( "summary result is " + str( num_found) + " so cx : " + str( cx) + " cy : " + 
					str(cy))	

			if( num_found == 2):		
				if( self.agree_center( self.auv.state[0] , cy , 0.1 ) ):
					self.ok_x = True
				else:
					self.ok_x = False
				if( self.agree_center( self.auv.state[1] , cx , 0.1 ) ):
					self.ok_y = True
				else:
					self.ok_y = False
				if( self.ok_y and self.ok_x):
					print( "OK_ALL")
					break
				elif( self.ok_y and not self.ok_x):
					self.auv.send_velocity([ cy * 0.5 , 0 , 0 , 0 , 0 , 0 , 0 , 0])
					print( "now ok only y axis of robot")
				elif( not self.ok_y and not self.ok_x):
					self.auv.send_velocity([ 0 ,  cx * -1 * 0.7 , 0 , 0 , 0 , 0 , 0 , 0])
					print( "now ok only x axis of robot")
				else:
					self.auv.send_velocity([ cy * 0.5 ,  cx * -1 * 0.7 , 0 , 0 , 0 , 0 , 0 , 0])
					print( "not ok any axis")
				
			else:
				print("Don't found anything")
		
		self.now_time = rospy.Time()
	
		print( "Finish time is " + str( ( self.now_time - self.start_time).to_sec()))

			
	def result_information( self ):		
		# service for call vision for find cash_in_your_chip	
		# how to use --> result = self.information( tast , request)
		# result must return cx1 cy1 cx2 cy2 area mode
		# new message maybe cx cy area mode
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

	def check_match_type( self , origin_1 , origin_2 , new_1 , new_2):
		distance_1_1 = self.find_distance( origin_1 , new_1 )
		distance_1_2 = self.find_distance( origin_1 , new_2 )
		if( distance_1_1 < distance_1_2): 
			return 1
		else: 
			return 0

	def agree_center( self , now , target , agree):
		distance = target - now
		if( agree * -1 < target and target < agree ) return True
		else return False
	
if __name__=='__main__':

#	def __init__( self , absolute_yaw , relative_x , relative_y , start_depth ):
	rospy.init_node("go_cash_in_your_chip")
	play_cash = go_cash_in_your_chip( 0 , 0 , 0 , -0.5)
	play_cash.main(2)
