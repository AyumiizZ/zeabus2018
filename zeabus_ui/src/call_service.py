#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
import math
import numpy
import cv2
from PyQt4 import QtGui , QtCore

class call_control_service( QtGui.QWidget ):
	def __init__(self , x , y):
		super( call_control_service , self).__init__()

#-------------------------------- for service ---------------------------------------------
		
		self.topic_service_yaw = "fix_abs_yaw"
		self.topic_service_xy = "fix_abs_xy"
		self.topic_service_distance = "fix_rel_xy"
		self.topic_service_depth = "fix_abs_depth"

		self.waiting_time = QtGui.QLineEdit("1")
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
		self.value_service_yaw = QtGui.QLineEdit()
		self.value_service_yaw.setValidator( QtGui.QDoubleValidator() )
		self.value_service_yaw.setAlignment( QtCore.Qt.AlignCenter )
		self.status_service_yaw = "Unknow"		

		self.button_service_xy = QtGui.QPushButton("call")
		self.value_service_xy = QtGui.QLineEdit()
		self.value_service_xy.setValidator( QtGui.QDoubleValidator() )
		self.value_service_xy.setAlignment( QtCore.Qt.AlignCenter )
		self.status_service_xy = "Unknow"		

		self.button_service_distance = QtGui.QPushButton("call")
		self.value_service_distance = QtGui.QLineEdit()
		self.value_service_distance.setValidator( QtGui.QDoubleValidator() )
		self.value_service_distance.setAlignment( QtCore.Qt.AlignCenter )
		self.status_service_distance = "Unknow"		

		self.button_service_depth = QtGui.QPushButton("call")
		self.value_service_depth = QtGui.QLineEdit()
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
		self.vertically_value.addWidget( self.value_service_xy )
		self.vertically_value.addStretch()
		self.vertically_value.addWidget( self.value_service_depth )
		self.vertically_value.addStretch()
		self.vertically_value.addWidget( self.value_service_yaw )
		self.vertically_value.addStretch()
		self.vertically_value.addWidget( self.value_service_distance )

		self.vertically_button = QtGui.QVBoxLayout()	
		self.vertically_button.addWidget( self.button_service_xy )
		self.vertically_button.addStretch()
		self.vertically_button.addWidget( self.button_service_depth )
		self.vertically_button.addStretch()
		self.vertically_button.addWidget( self.button_service_yaw )
		self.vertically_button.addStretch()
		self.vertically_button.addWidget( self.button_service_distance )

		self.vertically_status = QtGui.QVBoxLayout()	
		self.vertically_status.addWidget( QtGui.QLabel( self.status_service_xy ))
		self.vertically_status.addStretch()
		self.vertically_status.addWidget( QtGui.QLabel( self.status_service_depth ))
		self.vertically_status.addStretch()
		self.vertically_status.addWidget( QtGui.QLabel( self.status_service_yaw ))
		self.vertically_status.addStretch()
		self.vertically_status.addWidget( QtGui.QLabel( self.status_service_distance ))

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

		self.setLayout( self.all_vertically )

	def update_service(self):
		print("Welcome to test service")
		
		try:
			print("Call service yaw")
			rospy.wait_for_service( self.topic_service_yaw , timeout = 2.0)
			self.status_service_yaw = "Available"
		except:
			print("Failure")
			self.status_service_yaw = "Don't Available!"

		try:
			print("Call service xy")
			rospy.wait_for_service( self.topic_service_xy , timeout = 2.0)
			self.status_service_xy = "Available"
		except:
			print("Failure")
			self.status_service_xy = "Don't Available!"

		try:
			print("Call service depth")
			rospy.wait_for_service( self.topic_service_depth , timeout = 2.0)
			self.status_service_depth = "Available"
		except:
			print("Failure")
			self.status_service_depth = "Don't Available"

		try:
			print("Call service distance")
			rospy.wait_for_service( self.topic_servic_distance , timeout = 2.0)
			self.status_service_distance = "Available"
		except:
			print("Failure")
			self.status_service_distance = "Don't Available"
