import sys

from PyQt4 import QtGui

class main_window():
	def __init__(self):
		application = QtGui.QApplication(sys.argv) # handlef system-wide and application-wide
		
		window = QtGui.QWidget()

		button_01 = QtGui.QPushButton("Button01")
		button_02 = QtGui.QPushButton("Button02")
		button_03 = QtGui.QPushButton("Button03")

		vbox = QtGui.QVBoxLayout() # manage verticle layout

		vbox.addWidget(button_01) # add a widget to the BoxLayout
		
		vbox.addStretch() # creates empty stretchable box

		vbox.addWidget(button_02)

		vbox.addStretch() # creates empty stretchable box

		vbox.addWidget(button_03)

		window.setWindowTitle("Main Window")

		window.setGeometry(0 , 0 , 200 , 400) # set origin window

		window.setLayout( vbox )

		hbox = QtGui.QHBoxLayout() # manage horizontal layout

		button_04 = QtGui.QPushButton("Button04")
		button_05 = QtGui.QPushButton("Button05")
		button_06 = QtGui.QPushButton("Button06")

		hbox.addWidget(button_04)

		hbox.addStretch()
		
		hbox.addWidget(button_05)

		window.setLayout( hbox ) # don't show hbox because have set vbox	
		
		window.show()
		
		sys.exit(application.exec_())

if __name__ == "__main__":
	test = main_window()
