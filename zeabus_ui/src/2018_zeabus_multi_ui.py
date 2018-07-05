#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
from PyQt4 import QtGui
from flight_display import draw_flight_display
from camera_display import draw_picture_camera

#set window size
global pixel_x , pixel_y, width, height
pixel_x = 1400
pixel_y = 1000
width = 0
height = 0

class main_window_ui( QtGui.QMainWindow):
	def __init__(self , parent=None):
		super( main_window_ui , self).__init__(parent)

		global pixel_x , pixel_y, width, height
		self.x = pixel_x
		self.y = pixel_y

		self.multiple_document_interface = QtGui.QMdiArea()

		self.setCentralWidget( self.multiple_document_interface )


#		self.tool_bar = self.addToolBar("Option")

#---------------Add button for flight display button--------------------------

##		self.flight_display_button = QtGui.QPushButton("Flight Display")
#		self.flight_display_button.setCheckable(True)
#		self.flight_display_button.toggle()
##		self.flight_display_button.clicked.connect( self.show_flight_display )
##		self.tool_bar.addWidget( self.flight_display_button )

		self.tool_bar = self.menuBar()
		
		self.flight_display = self.tool_bar.addMenu("Flight Display")
		self.flight_display.addAction("Top")
		self.flight_display.addAction("Bottom")
		self.flight_display.triggered[QtGui.QAction].connect(self.show_flight_display)

		self.camera_display = self.tool_bar.addMenu("Camera Display")
		self.camera_display.addAction("Top")
		self.camera_display.addAction("Bottom")
		self.camera_display.triggered[QtGui.QAction].connect(self.show_camera_display)

		self.call_service = self.tool_bar.addMenu("service")
			
		self.setGeometry(0, 0, width, height)
		self.setWindowTitle( "ZEABUS UI 2018")
#		print( "main_window_ui.count first time is " + str(main_window_ui.count))

	def show_flight_display(self, message):
		if( message.text() == "Top"):
			flight_display = QtGui.QMdiSubWindow()
			flight_display.setWidget( draw_flight_display(self.x / 2 , self.y / 2 , 1) )
			flight_display.setGeometry(0 , 0 , self.x / 2, self.y / 2)
			flight_display.setWindowTitle( "flight display window")
			self.multiple_document_interface.addSubWindow( flight_display )
			flight_display.show()
		elif ( message.text() == "Bottom"):
			flight_display = QtGui.QMdiSubWindow()
			flight_display.setWidget( draw_flight_display(self.x / 2 , self.y / 2 , 2) )
			flight_display.setGeometry(0 , 0 , self.x / 2, self.y / 2)
			flight_display.setWindowTitle( "flight display window")
			self.multiple_document_interface.addSubWindow( flight_display )
			flight_display.show()

	def show_camera_display(self, message):
		if( message.text() == "Top"):
			camera_display = QtGui.QMdiSubWindow()
			camera_display.setWidget( draw_picture_camera(self.x / 2 , self.y / 2 , 1) )
			camera_display.setGeometry(0 , 0 , self.x / 2, self.y / 2)
			camera_display.setWindowTitle( "camera display window")
			self.multiple_document_interface.addSubWindow( camera_display )
			camera_display.show()
		elif ( message.text() == "Bottom"):
			camera_display = QtGui.QMdiSubWindow()
			camera_display.setWidget( draw_picture_camera(self.x / 2 , self.y / 2 , 2) )
			camera_display.setGeometry(0 , 0 , self.x / 2, self.y / 2)
			camera_display.setWindowTitle( "camera display window")
			self.multiple_document_interface.addSubWindow( camera_display )
			camera_display.show()

def main():
	global width, height
	application = QtGui.QApplication(sys.argv)
	
	screen_resolution = application.desktop().screenGeometry()
	width, height = screen_resolution.width() , screen_resolution.height()
	print("width : " + str(width) + "\theight : " + str(height))

	ui_window = main_window_ui()

	ui_window.show()

	sys.exit(application.exec_())

if __name__ == "__main__":
#	rospy.init_node('zeabus_ui', anonymous = True)
	rospy.init_node('2018_zeabus_ui', anonymous = True)
	main() 
