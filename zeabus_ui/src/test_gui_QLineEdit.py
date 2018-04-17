#!/usr/bin/env python
#please :setup nu tabstop=4

import sys

from PyQt4 import QtGui
from PyQt4 import QtCore
class main_window():
	def __init__(self):
		application = QtGui.QApplication(sys.argv)

		window = QtGui.QWidget()

		line01 = QtGui.QLineEdit() # have space for input
		line01.setValidator(QtGui.QIntValidator()) # set Rules for type of input
		line01.setMaxLength(4) # set the maximum number of character for input
		line01.setAlignment(QtCore.Qt.AlignRight) # can't use QtGui 
			#have AlignLeft AlignRight AlignCenter AlignJustify
		line01.setFont( QtGui.QFont("Arial" , 20))

		line02 = QtGui.QLineEdit()
		line02.setValidator(QtGui.QDoubleValidator(0.99 , 99.99 , 2))
		
		layout = QtGui.QFormLayout()
		layout.addRow("integer validator", line01)
		layout.addRow("Double validator", line02) # verticle
	
		line03 = QtGui.QLineEdit()
		line03.setInputMask('+99_9999_99999999')
		
		layout.addRow("input Mask" , line03)
		
		line04 = QtGui.QLineEdit()
		line04.textChanged.connect(self.textchanged) # this will go to function always 

		layout.addRow("Text Changed", line04)

		line05 = QtGui.QLineEdit()
		line05.setEchoMode(QtGui.QLineEdit.Password)

		layout.addRow("Password" , line05)

		line06 = QtGui.QLineEdit("Hello Python")
		line06.setReadOnly(True)

		layout.addRow("Read Only", line06)

		line05.editingFinished.connect(self.enterPress) # finish input

		window.setLayout(layout)
		window.setWindowTitle("PyQt")
		window.show()
		sys.exit(application.exec_())

	def textchanged(self, text):
		print( "contents of text box : "+text )

	def enterPress(self):
		print("edited")

if __name__=="__main__":
	test = main_window()
