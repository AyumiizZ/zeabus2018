import sys

from PyQt4 import QtGui

class main_window():
	def __init__(self):
		print("helllo")
		application = QtGui.QApplication(sys.argv)
		window = QtGui.QWidget()
	
		label_01 = QtGui.QLabel("Label 01")
		text_01 = QtGui.QLineEdit()

		label_02 = QtGui.QLabel("Label 02")
		text_02_01= QtGui.QLineEdit()
		text_02_02= QtGui.QLineEdit()
		
		fbox = QtGui.QFormLayout()

		fbox.addRow(label_01 , text_01)
	
		vbox = QtGui.QVBoxLayout()
	
		vbox.addWidget(text_02_01)
		vbox.addWidget(text_02_02)

		fbox.addRow(label_02 , vbox)

		hbox= QtGui.QHBoxLayout()

		r1 = QtGui.QRadioButton("Male")
		r2 = QtGui.QRadioButton("Female")

		hbox.addWidget(r1)
		hbox.addWidget(r2)
		hbox.addStretch()

		fbox.addRow(QtGui.QLabel("sex"), hbox)
		fbox.addRow(QtGui.QPushButton("Submit"), QtGui.QPushButton("Cancel"))
	
		window.setLayout(fbox)

		window.setWindowTitle("name_window")
		window.show()
	
		sys.exit(application.exec_())

if __name__ == '__main__':
	test = main_window()
