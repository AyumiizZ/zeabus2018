#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from gate import Gate
import constants as cons
from path2 import Path

class GatePath(object) :
    def __init__(self) :
        print '<===INIT GATE _PATH===>'
        self.aicontrol = AIControl()
        self.gate = Gate()
        self.path = Path()

    def run(self) :
        auv = self.aicontrol
        self.gate.run()
        #auv.driveY(0.5)
        self.path.run()
if __name__ == '__main__' :
    rospy.init_node('gate_path_node')
    gate_path = GatePath()
    gate_path.run()
