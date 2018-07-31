#!/usr/bin/python2

import	rospy, math, tf
from 	zeabus_controller.msg import point_xy
from 	zeabus_controller.srv import *

from	geometry_msgs.msg	import 	Twist, Pose, Point
from	nav_msgs.msg		import 	Odometry
from	std_msgs.msg		import	String, Float64, Bool

class	auv_control:

	def __init__(self , user):
	
		self.user 		= String(user)

		self.velocity 	= Twist()
		self.pose		= Pose()
		
		self.state		= [0 , 0 , 0 , 0 , 0 , 0]
		self.checkpoint = [0 , 0 , 0 , 0 , 0 , 0]

		rospy.Subscriber('/auv/state' , Odometry , self.listen_state)

		self.pub_velocity	= rospy.Publisher('/zeabus/cmd_vel' , Twist , queue_size=10)

		self.fix_abs_xy 	= rospy.ServiceProxy('/fix_abs_xy'		, fix_abs_xy)
		self.fix_rel_xy		= rospy.ServiceProxy('/fix_rel_xy'		, fix_rel_xy)
		self.fix_abs_yaw	= rospy.ServiceProxy('/fix_abs_yaw'		, fix_abs_yaw)
		self.fix_rel_yaw	= rospy.ServiceProxy('/fix_rel_yaw'		, fix_abs_yaw)
		self.fix_abs_depth	= rospy.ServiceProxy('/fix_abs_depth'	, fix_abs_depth)
		self.fix_rel_depth	= rospy.ServiceProxy('/fix_rel_depth'	, fix_abs_depth)
		self.ok_position	= rospy.ServiceProxy('/ok_position'		, ok_position)
		self.know_target 	= rospy.ServiceProxy('/know_target' 	, target_service)

	def find_target(self , data_type ):
		result = self.know_target( String( data_type ))
		return [ result.target_01 , result.target_02 ]

	def	relative_xy(self , relative_x , relative_y):
		self.fix_rel_xy( relative_x , relative_y , self.user)
		print( "--------------------------->go relative x : " + str(relative_x) 
                                                    + " and y : " + str( relative_y ) )

	def absolute_xy(self , absolute_x , absolute_y):
		self.fix_abs_xy( absolute_x , absolute_y , self.user)
		print( "--------------------------->go fix point : " + str( absolute_x ) 
                                                    + " and y : " + str( absolute_y ) )

	def	absolute_yaw(self , yaw):
		self.fix_abs_yaw( yaw , self.user )
		print "---------------------------->go fix yaw : " + str( yaw )

	def relative_yaw(self , yaw):
		self.fix_rel_yaw( yaw , self.user )
		print "---------------------------->go relative yaw : " + str( yaw )

	def	relative_depth( self , depth):
		self.fix_rel_depth( depth , self.user )
		print "---------------------------->go relative depth : " + str( depth )

	def absolute_depth( self , depth):
		self.fix_abs_depth( depth , self.user )
		print "---------------------------->go fix depth : " + str( depth )

	def check_position( self , data_type , adding_error ):
		result = self.ok_position( String ( data_type ) , adding_error , self.user)
		print( "--------------------------->you check " + data_type 
                                                    + " and answer is " + str(result.ok) )
		return result.ok

	def	listen_state( self , message):
		self.pose = message.pose.pose
		pose = self.pose

		temp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		euler_angular = tf.transformations.euler_from_quaternion(temp)

		self.state[0] = pose.position.x
		self.state[1] = pose.position.y
		self.state[2] = pose.position.z

		self.state[3] = euler_angular[0]
		self.state[4] = euler_angular[1]
		self.state[5] = euler_angular[2]

	def send_velocity( self , velocity):
		print("------------------------>you send velocity only")
		self.velocity.linear.x = velocity[0]
		self.velocity.linear.y = velocity[1]
		self.velocity.linear.z = velocity[2]

		self.velocity.angular.x = velocity[3]
		self.velocity.angular.y = velocity[4]
		self.velocity.angular.z = velocity[5]
	
		for i in range(4):
			self.pub_velocity.publish(self.velocity)

	def continue_move( self , velocity , amont_round , time_sleep):
		print("------------------------>you send velocity continue ")
		self.velocity.linear.x = velocity[0]
		self.velocity.linear.y = velocity[1]
		self.velocity.linear.z = velocity[2]

		self.velocity.angular.x = velocity[3]
		self.velocity.angular.y = velocity[4]
		self.velocity.angular.z = velocity[5]

		for i in range( amont_round ):
			print("---------> count is " + str( i ))
			self.pub_velocity.publish( self.velocity )
			rospy.sleep( time_sleep )
			
