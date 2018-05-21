#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
import math
import numpy
import cv2
from PyQt4 import QtGui , QtCore
from zeabus_controller.srv import *
from zeabus_controller.msg import point_xy

class call_control_service( QtGui.QWidget ):
	def __init__(self , x , y):
		super( call_control_service , self).__init__()

#-------------------------------- for service ---------------------------------------------

		self.time = 2.0
		self.value_xy_x = 0.0
		self.value_xy_y = 0.0
		self.value_distance_x = 0.0
		self.value_distance_y = 0.0
		self.value_depth = -2.0
		self.value_yaw = 0.0
		
		self.topic_service_yaw = '/fix_abs_yaw'
		self.topic_service_xy = '/fix_abs_xy'
		self.topic_service_distance = '/fix_rel_xy'
		self.topic_service_depth = '/fix_abs_depth'

		self.service_xy = rospy.ServiceProxy( self.topic_service_xy , fix_abs_xy )
		self.service_yaw = rospy.ServiceProxy( self.topic_service_yaw , fix_abs_yaw )
		self.service_depth = rospy.ServiceProxy( self.topic_service_depth , fix_abs_depth )
		self.service_distance = rospy.ServiceProxy( self.topic_service_distance , fix_rel_xy )

		self.waiting_time = QtGui.QLineEdit( str(self.time) )
		self.waiting_time.setAlignment( QtCore.Qt.AlignCenter )
		self.waiting_time.setValidator( QtGui.QDoubleValidator() )
		self.waiting_time_button = QtGui.QPushButton("Update")
		self.horizontal_general = QtGui.QHBoxLayout()
		self.horizontal_general.addWidget( QtGui.QLabel("Time waiting service"))
		self.horizontal_general.addStretch()
		self.horizontal_general.addWidget( self.waiting_time )
		self.horizontal_general.addStretch()
		self.horizontal_general.addWidget( self.waiting_time_button )

		self.button_service_yaw = QtGui.QPushButton("call")
		self.value_service_yaw = QtGui.QLineEdit( str(self.value_yaw) )
		self.value_service_yaw.setValidator( QtGui.QDoubleValidator() )
		self.value_service_yaw.setAlignment( QtCore.Qt.AlignCenter )
		self.status_service_yaw = "Unknow"		

		self.value_service_xy_x = QtGui.QLineEdit( str(self.value_xy_x) )
		self.value_service_xy_x.setValidator( QtGui.QDoubleValidator() )
		self.value_service_xy_x.setAlignment( QtCore.Qt.AlignCenter )
		self.value_service_xy_y = QtGui.QLineEdit( str(self.value_xy_y) )
		self.value_service_xy_y.setValidator( QtGui.QDoubleValidator() )
		self.value_service_xy_y.setAlignment( QtCore.Qt.AlignCenter )

		self.button_service_xy = QtGui.QPushButton("call")
		self.value_service_xy = QtGui.QHBoxLayout()
		self.value_service_xy.addWidget( self.value_service_xy_x )
		self.value_service_xy.addStretch()
		self.value_service_xy.addWidget( self.value_service_xy_y )
		self.status_service_xy = "Unknow"		

		self.value_service_distance_x = QtGui.QLineEdit( str(self.value_distance_x) )
		self.value_service_distance_x.setValidator( QtGui.QDoubleValidator() )
		self.value_service_distance_x.setAlignment( QtCore.Qt.AlignCenter )
		self.value_service_distance_y = QtGui.QLineEdit( str(self.value_distance_y) )
		self.value_service_distance_y.setValidator( QtGui.QDoubleValidator() )
		self.value_service_distance_y.setAlignment( QtCore.Qt.AlignCenter )

		self.button_service_distance = QtGui.QPushButton("call")
		self.value_service_distance = QtGui.QHBoxLayout()
		self.value_service_distance.addWidget( self.value_service_distance_x )
		self.value_service_distance.addStretch()
		self.value_service_distance.addWidget( self.value_service_distance_y )
		self.status_service_distance = "Unknow"		

		self.button_service_depth = QtGui.QPushButton("call")
		self.value_service_depth = QtGui.QLineEdit( str(self.value_depth) )
		self.value_service_depth.setValidator( QtGui.QDoubleValidator() )
		self.value_service_depth.setAlignment( QtCore.Qt.AlignCenter )
		self.status_service_depth = "Unknow"	

		self.vertically_topic = QtGui.QVBoxLayout()	
		self.vertically_topic.addWidget( QtGui.QLabel(self.topic_service_xy) )
		self.vertically_topic.addStretch()
		self.vertically_topic.addWidget( QtGui.QLabel(self.topic_service_depth) )
		self.vertically_topic.addStretch()
		self.vertically_topic.addWidget( QtGui.QLabel(self.topic_service_yaw) )
		self.vertically_topic.addStretch()
		self.vertically_topic.addWidget( QtGui.QLabel(self.topic_service_distance) )

		self.vertically_value = QtGui.QVBoxLayout()	
		self.vertically_value.addLayout( self.value_service_xy )
		self.vertically_value.addStretch()
		self.vertically_value.addWidget( self.value_service_depth )
		self.vertically_value.addStretch()
		self.vertically_value.addWidget( self.value_service_yaw )
		self.vertically_value.addStretch()
		self.vertically_value.addLayout( self.value_service_distance )

		self.vertically_button = QtGui.QVBoxLayout()	
		self.vertically_button.addWidget( self.button_service_xy )
		self.vertically_button.addStretch()
		self.vertically_button.addWidget( self.button_service_depth )
		self.vertically_button.addStretch()
		self.vertically_button.addWidget( self.button_service_yaw )
		self.vertically_button.addStretch()
		self.vertically_button.addWidget( self.button_service_distance )

		self.status_label_xy = QtGui.QLabel( self.status_service_xy , self)
		self.status_label_distance = QtGui.QLabel( self.status_service_distance , self)
		self.status_label_yaw = QtGui.QLabel( self.status_service_yaw , self)
		self.status_label_depth = QtGui.QLabel( self.status_service_depth , self)

		self.vertically_status = QtGui.QVBoxLayout()	
		self.vertically_status.addWidget( self.status_label_xy)
		self.vertically_status.addStretch()
		self.vertically_status.addWidget( self.status_label_depth )
		self.vertically_status.addStretch()
		self.vertically_status.addWidget( self.status_label_yaw )
		self.vertically_status.addStretch()
		self.vertically_status.addWidget( self.status_label_distance )

		self.horizontal_service = QtGui.QHBoxLayout()
		self.horizontal_service.addLayout( self.vertically_topic )	
		self.horizontal_service.addStretch()
		self.horizontal_service.addLayout( self.vertically_value )	
		self.horizontal_service.addStretch()
		self.horizontal_service.addLayout( self.vertically_button )	
		self.horizontal_service.addStretch()
		self.horizontal_service.addLayout( self.vertically_status )	

		self.all_vertically	= QtGui.QVBoxLayout()
		self.all_vertically.addLayout( self.horizontal_general )
		self.all_vertically.addStretch()
		self.all_vertically.addLayout( self.horizontal_service )

		self.waiting_time.textChanged.connect( self.update_time )
		self.value_service_xy_x.textChanged.connect( self.update_value_xy_x)
		self.value_service_xy_y.textChanged.connect( self.update_value_xy_y)
		self.value_service_distance_x.textChanged.connect( self.update_value_distance_x)
		self.value_service_distance_y.textChanged.connect( self.update_value_distance_y)
		self.value_service_depth.textChanged.connect( self.update_value_depth)
		self.value_service_yaw.textChanged.connect( self.update_value_yaw)

		self.button_service_xy.clicked.connect( self.call_xy )
		self.button_service_distance.clicked.connect( self.call_distance )
		self.button_service_yaw.clicked.connect( self.call_yaw )
		self.button_service_depth.clicked.connect( self.call_depth )
		self.waiting_time_button.clicked.connect( self.update_service )

		self.setLayout( self.all_vertically )

	def update_service(self):
		print("Welcome to test service")
		
		try:
			print("Call service xy")
			rospy.wait_for_service( self.topic_service_xy , timeout = self.time)
			self.status_service_xy = "Available"
		except:
			print("Failure")
			self.status_service_xy = "Don't Available!"
		self.status_label_xy.setText(self.status_service_xy)

		try:
			print("Call service depth")
			rospy.wait_for_service( self.topic_service_depth , timeout = self.time)
			self.status_service_depth = "Available"
		except:
			print("Failure")
			self.status_service_depth = "Don't Available!"
		self.status_label_depth.setText(self.status_service_depth)

		try:
			print("Call service yaw")
			rospy.wait_for_service( self.topic_service_yaw , timeout = self.time)
			self.status_service_yaw = "Available"
		except:
			print("Failure")
			self.status_service_yaw = "Don't Available!"
		self.status_label_yaw.setText(self.status_service_yaw)

		try:
			print("Call service distance")
			rospy.wait_for_service( self.topic_service_distance , timeout = self.time)
			self.status_service_distance = "Available"
		except:
			print("Failure")
			self.status_service_distance = "Don't Available!"
		self.status_label_distance.setText( self.status_service_distance)
		

	def update_value_xy_x(self , value):
		self.value_xy_x = value
		print("self.value_xy_x is " + str( round( float(self.value_xy_x), 2)))

	def update_value_xy_y(self , value):
		self.value_xy_y = value
		print("self.value_xy_y is " + str( round( float(self.value_xy_y), 2)))

	def update_value_distance_x(self , value):
		self.value_distance_x = value

	def update_value_distance_y(self , value):
		self.value_distance_y = value

	def update_value_depth(self , value):
		self.value_depth = value

	def update_value_yaw(self , value):
		self.value_yaw = value

	def update_time(self , value):
		self.time = float(value)

	def call_xy(self):
		result = "what"
		try:
			print("Call Service xy")
			result = self.service_xy( float(self.value_xy_x) , float(self.value_xy_y))
			self.status_service_xy = "Success"
		except:
			print("Result is "+result)	
			self.status_service_xy = "Not Success"
		self.status_label_xy.setText(self.status_service_xy)

	def call_depth(self):
		result = "what"
		try:
			print("Call Service depth")
			result = self.service_depth( float(self.value_depth))
			self.status_service_depth = "Success"
		except:
			print("Result is "+result)	
			self.status_service_depth = "Not Success"
		self.status_label_depth.setText(self.status_service_depth)

	def call_distance(self):
		result = "what"
		try:
			print("Call Service distance")
			result = self.service_distance( float(self.value_distance_x) 
										, float(self.value_distance_y) )
			self.status_service_distance = "Success"
		except:
			print("Result is "+result)	
			self.status_service_distance = "Not Success"
		self.status_label_distance.setText( self.status_service_distance)

	def call_yaw(self):
		result = "what"
		try:
			print("Call Service depth")
			result = self.service_yaw( float(self.value_yaw) )
			self.status_service_yaw = "Success"
		except:
			print("Result is "+result)	
			self.status_service_yaw = "Not Success"
		self.status_label_yaw.setText(self.status_service_yaw)
