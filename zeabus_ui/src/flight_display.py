#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
from PyQt4 import QtGui , QtCore
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class draw_flight_display(QtGui.QWidget):

	def __init__(self, x , y):
		super( draw_flight_display , self).__init__()
		rospy.init_node('flight_display', anonymous = True)
		self.pixel_x = x
		self.pixel_y = y

# subscribe topic
		rospy.Subscriber('/auv/state', Odometry, self.get_current_state)

# style have Qt.SolidLine , Qt.DashLine , Qt.DotLine 
# and Qt.DashDotLine , Qt.DashDotDotLine , Qt.CustomDashLine
		self.black_pen = QtGui.QPen()
		self.black_pen.setStyle( QtCore.Qt.SolidLine )
		self.black_pen.setWidth( 3 )
		self.black_pen.setBrush( QtCore.Qt.black )

	def paintEvent(self, event):
		drawing = QtGui.QPainter()
		drawing.begin(self)
		drawing.fillRect(0,0 , self.pixel_x , self.pixel_x, QtGui.QColor( QtCore.Qt.gray ) )
		drawing.fillRect(0,self.pixel_x -5 , self.pixel_x , 10 , QtGui.QColor( QtCore.Qt.red))
		drawing.setPen( self.black_pen )
		drawing.setFont( QtGui.QFont('Times' , 20))
		drawing.drawText(20 , self.pixel_x + 40 , "X = ")
		drawing.end()

	def get_current_state(self, message ):
		print("receive value")

	def draw_init_ui(self):
		self.setGeometry(0 , 0 , 600 , 800)
		self.setWindowTitle("flight display")
		self.show
