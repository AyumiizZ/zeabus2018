#!/usr/bin/env python
#plesase setup nu tabstop=4

import sys

from PyQt4 import QtGui , QtCore

class gui_mainwindow():
	def __init__(self):
		self.app = QtGui.QApplication(sys.argv)

		w = QtGui.QWidget()
#		w = QtGui.QDialog() # for test botton	
		b = QtGui.QLabel(w)

		b.setText("Hello World!") # set text in GUI

		w.setGeometry(100 , 100 , 200 , 50) # xpos , ypos , width , height

		b.move(25 , 0) # set location 0,0 is top left (x , y)

		w.setWindowTitle("w_setwindow") # set text show on Title? top GUI 

		b1 = QtGui.QPushButton(w)
		b1.setText("Button1")
		b1.move(25,25)
		QtCore.QObject.connect(b1, QtCore.SIGNAL("clicked()"), self.have_clicked)

		w.show() # for show GUI
	
		sys.exit(self.app.exec_())

	def have_clicked(self):
		print("click Button1")
#		self.app_clicked = QtGui.QApplication(sys.argv)

		b1_window = QtGui.QWidget()

		b1_text = QtGui.QLabel(b1_window)
		b1_text.setText("Hello botton 01")
		
		b1_window.setGeometry(200 , 200 , 100 ,30)
		b1_text.move(50 , 50)
		
		b1_window.setWindowTitle("Window button from 01")

		b1_window.show()
		print("last line of have_clicked")
#		sys.exit(self.app.exec_()) # can't use same variable
#		sys.exit(self.app_clicked.exec_())



if __name__=="__main__":
	test = gui_mainwindow()
