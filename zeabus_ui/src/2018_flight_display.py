#!/usr/bin/env python
#please :set nu tabstop=4

import sys

import rospy
import math , pygtk
import gtk, gobject, cairo

from PyQt4 import QtGui
from gtk import gdk
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, Joy

def main_window( Widget):
	window = QtGui.QWidget()
	
	window.setLayout( Widget )

	window.setWindowTitle("2018 Display Fligther")

	window.show()

	print( 'last line of main_window' )

if __name__ == "__main__":
	rospy.init_node("node")	# init node name
#	rospy.Subscriber('/auv/state' , Odometry , getState) # for receive current state

	application = QtGui.QApplication(sys.argv)

	test_01 = QtGui.QLabel("test 01")
	test_03 = QtGui.QLabel("test 03")
	test_02 = QtGui.QFormLayout()
	test_02.addRow(test_01 , test_03)

	window = QtGui.QWidget()
	window.setLayout( test_02 )
	window.setGeometry( 200 , 200 , 1000 , 600)
	window.setWindowTitle("2018 Display Fligther")
	window.show()

	sys.exit( application.exec_() )
