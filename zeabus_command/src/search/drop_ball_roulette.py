#!/usr/bin/python2

from control_auv		import auv_control
from zeabus_vision.srv	import vision_srv_roulette
from zeabus_vision.msg	import vision_roulette

class drop_roulette:

	def __init__( self , color_target ):

#------------------ setup handle for use control ------------------------------------
		self.auv = auv_control( "drop roulette")

#------------------ setup service for call data from vision -------------------------
		print "wait service /vision_roulette"
		self.information 	= rospy.ServiceProxy('vision_roulette' , vision_roulette)
		print "have service /vision_roulette"
#------------------------------------------------------------------------------------

		if color_target == "black":
			self.left = "green"
			self.target = "black"
			self.right = "red"	
		else if color_target == "green":
			self.left = "red"
			self.target = "green"
			self.right = "black"	
		else :
			self.left = "black"
			self.target = "red"
			self.right = "green"	
