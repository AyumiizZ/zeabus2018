#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
import math
import numpy
import cv2 
from PyQt4 import QtGui , QtCore
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

class draw_flight_display(QtGui.QWidget):
	def __init__(self, x , y, mode):
		super( draw_flight_display , self).__init__()
#		rospy.init_node('flight_display', anonymous = True)
#self.pixel_? is length in there axis
		self.pixel_x = x
		self.pixel_y = y

#self.position is x y z roll pitch yaw
		self.position = [0, 0, 0, 0, 0, 0]

#-----------------------------------------size of image-----------------------------------------
	# for center camera
		self.size_x_top = 1152
		self.size_y_top = 874
		self.point_for_center_x_top = 0 - ( ( self.size_x_top / 2 ) - ( self.pixel_y / 2 ) )
		self.point_for_center_y_top = 0 - ( ( self.size_y_top / 2 ) - ( self.pixel_y / 2 ) )

	# for bottom camera
		self.size_x_bottom = 1936
		self.size_y_bottom = 1216
		self.point_for_bottom_x_top = 0 - ( ( self.size_x_top / 2 ) - ( self.pixel_y / 2 ) )
		self.point_for_bottom_y_top = 0 - ( ( self.size_y_top / 2 ) - ( self.pixel_y / 2 ) )
#------------------------------------------------------------------------------------------------

# mode_picture 0 = not image, 1 = center camera, 2 = bottom camera
		self.mode_picture = mode
		self.center_image = None
		self.bottom_image = None

		self.show_type = 0

# subscribe topic
		rospy.Subscriber('/auv/state', Odometry, self.get_current_state)
		if( self.mode_picture == 1):
			rospy.Subscriber('/top/center/image_raw/compressed', 
								CompressedImage , self.get_center_image )
		elif (self.mode_picture == 2):
			rospy.Subscriber('/bottom/left/image_raw/compressed', 
								CompressedImage , self.get_bottom_image )

# style have Qt.SolidLine , Qt.DashLine , Qt.DotLine 
# and Qt.DashDotLine , Qt.DashDotDotLine , Qt.CustomDashLine
		self.black_pen = QtGui.QPen()
		self.black_pen.setStyle( QtCore.Qt.SolidLine )
		self.black_pen.setWidth( 3 )
		self.black_pen.setBrush( QtCore.Qt.black )

		self.center_pen = QtGui.QPen()
		self.center_pen.setStyle( QtCore.Qt.SolidLine )
		self.center_pen.setWidth( 2 )
		self.center_pen.setBrush( QtCore.Qt.magenta )

	def paintEvent(self, event):
#		print("paintEvent")
		drawing = QtGui.QPainter()
		drawing.begin(self)

# drawPixmap order argument mean point_x_top_left , point_y_top_left , image
#		drawing.drawPixmap(0 , 0 , QtGui.QPixmap("natural_image.jpg"))
		if( self.show_type == 1 ):
			drawing.drawPixmap( self.point_for_center_x_top , self.point_for_center_y_top,
								QtGui.QPixmap( self.center_image ))
		elif( self.show_type == 2 ):
			drawing.drawPixmap( self.point_for_bottom_x_top , self.point_for_bottom_y_top,
								QtGui.QPixmap( self.bottom_image ))

# fillRect order argument mean start_x , start_y , plus_x , plus_y , color
#		drawing.fillRect(0,0 , self.pixel_y , self.pixel_y, QtGui.QColor( QtCore.Qt.gray ) )
		drawing.fillRect(self.pixel_y - 5 , 0 , 5 , self.pixel_y , 
							QtGui.QColor( QtCore.Qt.red))
		drawing.fillRect(self.pixel_y , 0 , self.pixel_x , self.pixel_y , 
							QtGui.QColor( QtCore.Qt.white))

#drawing line order argumment mean start_point_x , stary_point_y , end_point_x , end_point_y	
		drawing.setPen( self.center_pen )
		drawing.drawLine(self.pixel_y/2-5 , self.pixel_y/2 , self.pixel_y/2+5 , self.pixel_y/2)
		drawing.drawLine(self.pixel_y/2 , self.pixel_y/2-5 , self.pixel_y/2 , self.pixel_y/2+5)
	# line x axis
		drawing.drawLine(self.pixel_y/2-60 , self.pixel_y/2 , self.pixel_y/2-50 , self.pixel_y/2 )
		drawing.drawLine( self.pixel_y/2+50 , self.pixel_y/2 , self.pixel_y/2+60 , self.pixel_y/2 )
	# line y axis
		drawing.drawLine(self.pixel_y/2 , self.pixel_y/2-60 , self.pixel_y/2 , self.pixel_y/2-50 )
		drawing.drawLine( self.pixel_y/2 , self.pixel_y/2+50 , self.pixel_y/2 , self.pixel_y/2+60 )

#drawText order argument mean point_x_left, point_y_bottom , text 
#	so color is same setPen and font size is same setFont
		drawing.setPen( self.black_pen )
		drawing.setFont( QtGui.QFont('Times' , 20))
		drawing.drawText(self.pixel_y + 20 , 30 , "current_state")
		drawing.setFont( QtGui.QFont('Times' , 16))
		drawing.drawText(self.pixel_y + 15 , 80 ,  "X\t= " + str( round( self.position[0],2 ) ) )
		drawing.drawText(self.pixel_y + 15 , 110 ,  "Y\t= " + str( round( self.position[1],2 ) ) )
		drawing.drawText(self.pixel_y + 15 , 140 , "Z\t= " + str( round( self.position[2],2 ) ) )
		drawing.drawText(self.pixel_y + 15 , 200 , "Roll\t= " + str( round( self.position[3],2 ) ) )
		drawing.drawText(self.pixel_y + 15 , 230 , "Pitch\t= " + str( round( self.position[4],2 ) ) )
		drawing.drawText(self.pixel_y + 15 , 260 , "Yaw\t= " + str( round( self.position[5],2 ) ) )

		drawing.end()

	def get_center_image(self, message):
		print("Recieve top image , flight display")
		temporary = numpy.fromstring( message.data, numpy.uint8)
		temporary = cv2.imdecode( temporary , 1)
		self.center_image = QtGui.QImage( temporary , temporary.shape[1],
									temporary.shape[0] , temporary.shape[1]*3,
									QtGui.QImage.Format_RGB888) 
		self.center_image = QtGui.QImage.rgbSwapped( self.center_image)
		self.show_type = 1
		self.update()

	def get_bottom_image(self, message):
		print("Recieve bottom image , flight display")
		temporary = numpy.fromstring( message.data, numpy.uint8)
		temporary = cv2.imdecode( temporary , 1)
		self.bottom_image = QtGui.QImage( temporary , temporary.shape[1],
									temporary.shape[0] , temporary.shape[1]*3,
									QtGui.QImage.Format_RGB888) 
		self.bottom_image = QtGui.QImage.rgbSwapped( self.bottom_image)
		self.show_type = 2
		self.update()

	def get_current_state(self, message ):
#		print("receive value")
		pose = message.pose.pose
		tmp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		ang = euler_from_quaternion(tmp)

		self.position[0] = pose.position.x
		self.position[1] = pose.position.y
		self.position[2] = pose.position.z

# for convert radian to degree
		temporary = ang[0]*180/math.pi
		self.position[3] = min([temporary-360,360-temporary,360+temporary,temporary], key=abs)		

		temporary = ang[1]*180/math.pi
		self.position[4] = min([temporary-360,360-temporary,360+temporary,temporary], key=abs)		

		temporary = ang[2]*180/math.pi
		self.position[5] = min([temporary-360,360-temporary,360+temporary,temporary], key=abs)		
		self.update()
