#!/usr/bin/env python
#please :set nu tabstop=4

import sys
import rospy
from PyQt4 import QtGui
from flight_display import draw_flight_display

#set window size
global pixel_x , pixel_y
pixel_x = 1400
pixel_y = 1000

class main_window_ui( QtGui.QMainWindow):
	def __init__(self , parent=None):
		super( main_window_ui , self).__init__(parent)

		global pixel_x , pixel_y
		self.x = pixel_x
		self.y = pixel_y

		self.multiple_document_interface = QtGui.QMdiArea()

		self.setCentralWidget( self.multiple_document_interface )

		self.tool_bar = self.addToolBar("Open New Window")

#---------------Add button for flight display button--------------------------

		self.flight_display_button = QtGui.QPushButton("Flight Display")
		self.flight_display_button.setCheckable(True)
		self.flight_display_button.toggle()
		self.flight_display_button.clicked.connect( self.show_flight_display )
		self.tool_bar.addWidget( self.flight_display_button )
		
		self.setGeometry(0, 0, self.x, self.y)
		self.setWindowTitle( "ZEABUS UI 2018")
#		print( "main_window_ui.count first time is " + str(main_window_ui.count))

	def show_flight_display(self):
#		print( self.flight_display_button.isChecked())
		if not self.flight_display_button.isChecked():
			print("button pressed")
			flight_display = QtGui.QMdiSubWindow()

			flight_display.setWidget( draw_flight_display(700 , self.y) )

			flight_display.setGeometry(0 , 0 , 700, self.y)

			flight_display.setWindowTitle( "flight display window")

			self.multiple_document_interface.addSubWindow( flight_display )
			flight_display.show()
		else:
			print("button released")


def main():
	application = QtGui.QApplication(sys.argv)
	
	ui_window = main_window_ui()

	ui_window.show()

	sys.exit(application.exec_())

if __name__ == "__main__":
#	rospy.init_node('zeabus_ui', anonymous = True)
	main() 
