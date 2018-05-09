#!/usr/bin/env python2
'''
    File name: zeabus_handle_node.py
    Author: zeabus2018
    Date created: 2018/04/26
    Python Version: 2.7
'''
import rospy
import rosnode
import os
import sys
from PyQt4 import QtGui, QtCore


class NodeHandle(QtGui.QWidget):
    def __init__(self, x, y, mode):
        super(NodeHandle, self).__init__()
        self.nodes = []
        self.status = []

    def node_status(self):
        tmp = rosnode.get_node_names()
        tmp += self.nodes
        tmp = set(tmp)
        tmp = list(tmp)
        self.nodes = sorted(tmp)
        size = len(self.nodes)
        self.status = [True] * size

        for i in range(size):
            try:
                self.status[i] = rosnode.rosnode_ping(
                    self.nodes[i], max_count=1)
            except Exception:
                self.status[i] = False


class Window(QtGui.QMainWindow):

    def __init__(self):
        self.app = QtGui.QApplication(sys.argv)
        super(Window, self).__init__()
        screen_resolution = self.app.desktop().screenGeometry() 
        width, height = screen_resolution.width(), screen_resolution.height()
        self.widget = QtGui.QWidget()
        self.show_service_display()

    def show_service_display(self):
        


        service_display.setGeometry(0, 0, self.x / 2, self.y / 2)
        self.multiple_document_interface.addSubWindow(service_display)
        service_display.show()

    def run(self):
        GUI = Window()
        self.show()
        sys.exit(self.app.exec_())


def main():
    w = Window()
    w.run()


if __name__ == '__main__':
    rospy.init_node('zeabus_handle_node', anonymous=False)
