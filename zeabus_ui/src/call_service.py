#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
import math
import numpy
import cv2
from PyQt4 import QtGui , QtCore

class call_control_service( QtGui.QWidget ):
	def __init__(self):
		super( call_control_service , self).__init__()
		
