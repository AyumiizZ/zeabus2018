#!/usr/bin/env python
#please :set nu tabstop=4

import sys
from PyQt4 import QtGui

class main_window_ui( QtGui.QMainWindow):
	def __init__(self , parent=None):
		super( main_window_ui , self).__init__(parent)
		
		self.flight_display_window = QtGui.QMdiArea()

		self.setCentralWidget( self.flight_display_window )

		menu_bar = self.menuBar()

		print( "main_window_ui.count first time is " + str(main_window_ui))


def main():
	application = QtGui.QApplication(sys.argv)
	
	ui_window = main_window_ui()

	ui_window.show()

	sys.exit(application.exec_())

if __name__ == "__main__":
	main() 
