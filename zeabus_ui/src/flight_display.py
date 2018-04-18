#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
import math
from PyQt4 import QtGui , QtCore
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class draw_flight_display(QtGui.QWidget):
	def __init__(self, x , y):
		super( draw_flight_display , self).__init__()
		rospy.init_node('flight_display', anonymous = True)
		self.pixel_x = x
		self.pixel_y = y
#self.position is x y z roll pitch yaw
		self.position = [0, 0, 0, 0, 0, 0]

# subscribe topic
		rospy.Subscriber('/auv/state', Odometry, self.get_current_state)

# style have Qt.SolidLine , Qt.DashLine , Qt.DotLine 
# and Qt.DashDotLine , Qt.DashDotDotLine , Qt.CustomDashLine
		self.black_pen = QtGui.QPen()
		self.black_pen.setStyle( QtCore.Qt.SolidLine )
		self.black_pen.setWidth( 3 )
		self.black_pen.setBrush( QtCore.Qt.black )

	def paintEvent(self, event):
		print("paint")
		drawing = QtGui.QPainter()
		drawing.begin(self)
		drawing.fillRect(0,0 , self.pixel_x , self.pixel_x, QtGui.QColor( QtCore.Qt.gray ) )
		drawing.fillRect(0,self.pixel_x -5 , self.pixel_x , 10 , QtGui.QColor( QtCore.Qt.red))
		drawing.setPen( self.black_pen )
		drawing.setFont( QtGui.QFont('Times' , 20))
		drawing.drawText(20 , self.pixel_x + 40 , "X = " + str(self.position[0]))
		drawing.end()

	def get_current_state(self, message ):
		print("receive value")
		pose = message.pose.pose
		tmp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		ang = euler_from_quaternion(tmp)

		self.position[0] = pose.position.x
		self.position[1] = pose.position.y
		self.position[2] = pose.position.z
		self.position[3] = ang[0]
		self.position[4] = ang[1]

		yaw_tmp = ang[2]*180/math.pi

		self.position[5] = min([yaw_tmp-360,360-yaw_tmp,360+yaw_tmp,yaw_tmp], key=abs)		
		self.update()

	def draw_init_ui(self):
		self.setGeometry(0 , 0 , 600 , 800)
		self.setWindowTitle("flight display")
		self.show
