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

import sys
from PyQt4.QtGui import *
from PyQt4.QtCore import *


class NodeHandle(QtGui.QWidget):
    def __init__(self):
        super(NodeHandle, self).__init__()
        self.nodes = []
        self.status = []

    def get_node_status(self):
        size = len(self.nodes)
        self.status = [True] * size

        for i in range(size):
            try:
                self.status[i] = rosnode.rosnode_ping(
                    self.nodes[i], max_count=1)
            except Exception:
                self.status[i] = False

    def get_node(self):
        tmp = rosnode.get_node_names()
        # tmp = self.nodes
        tmp = set(tmp)
        tmp = list(tmp)
        self.nodes = sorted(tmp)
        size = len(self.nodes)
        return self.nodes

    def kill_node(self, name):
        try:
            rosnode.kill_nodes([str(name)])
            return True
        except:
            return False


class myListWidget(QListWidget):
    def __init__(self):
        super(myListWidget, self).__init__()
        self.setWindowTitle('ZEABUS HANDLE NODES')
        self.itemClicked.connect(self.Clicked)
        self.nh = NodeHandle()
        self.fetch_node2list()
        self.resize(500, 500)

        self.btn = QPushButton('refresh', self)
        self.btn.move(200, 450)
        self.btn.clicked.connect(self.refresh_on_click)

    def Clicked(self, item):
        msg = QMessageBox()
        node_name = item.text()
        msg.setWindowTitle("Kill Node: ")
        msg.setText("Do u want to kill node: " + node_name)
        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        retval = msg.exec_()
        print(retval)
        if retval == 1024:
            res = self.nh.kill_node(node_name)
            print("Result: ", res)
        else:
            print('Cancel kill node')

        self.fetch_node2list()

    def fetch_node2list(self):
        self.clear()
        nodes = self.nh.get_node()
        if nodes is not None:
            for n in nodes:
                self.addItem(n)

    def refresh_on_click(self):
        self.fetch_node2list()


def main():
    app = QApplication(sys.argv)

    listWidget = myListWidget()
    listWidget.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    rospy.init_node('zeabus_handle_node', anonymous=True)
    main()
