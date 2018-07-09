#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from gate_qua2 import Gate
import constants as cons
from marker2 import Marker

class Qualify(object):
    def __init__(self):
        print '<===INIT QUALIFY===>'
        self.aicontrol = AIControl()
        self.marker = Marker()
        self.gate = Gate()

    def run(self) :
        auv = self.aicontrol
        auv.depthAbs(cons.QUALIFY_DEPTH)
        auv.driveX(5)
        self.marker.run()
        self.gate.run()
        auv.driveX(5)

if __name__=='__main__':
    rospy.init_node('qualifying_node')
    qualify = Qualify()
    qualify.run()
