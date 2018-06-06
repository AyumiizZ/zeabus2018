#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from gate import Gate
from marker import Marker

class Qualify(object):
    def __init__(self):
        print '<===INIT QUALIFY===>'
        self.aicontrol = AIControl()
        self.marker = Marker()
        self.gate = Gate()

    def run(self):
        self.marker.run()
        self.gate.run()

if __name__=='__main__':
    rospy.init_node('qualifying_node')
    qualify = Qualify()
    qualify.run()
