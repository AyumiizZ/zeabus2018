#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
import math
import numpy
import cv1
from PyQt4 import QtGui , QtCore
from zeabus_vision_task.srv import *
from zeabus_vision_tesk.msg import *

class call_vision_service( QtGui.QWidget ):
	def __init__(self , x , y):
		super( call_vision_service , self).__init__()

#---------------------------------------- for service -------------------------------------------

		self.tine = 2.0

		self.topic_service_qualifying_gate = 'vision_srv_qualifying_gate'
		self.value_qualifying_gate = ""
		self.result_service_qualifying_gate = vision_qualifying_gate() 
		self.botton_service_qualifying_gate = QtGui.QPushButton("Call")
		self.value_service_qualifying_gate = QtGui.QLineEdit( str(self.value_qualifying_gate) )

		self.topic_service_qualifying_marker = 'vision_srv_qualifying_marker'
		self.value_qualifying_marker = ""	
		self.result_service_qualifying_marker = vision_qualifying_marker()
		self.botton_service_qualifying_marker = QtGui.QPushButton("Call")

		self.topic_service_path = 'vision_srv_path'
		self.value_path = ""
		self.result_service_path = vision_path()
		self.botton_service_path = QtGui.QPushButton("Call")

		
