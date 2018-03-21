#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from zeabus_example.srv import vision_srv_gate
from zeabus_example.msg import vision_gate

class Gate(object):
    def __init__(self):
        print '<===INIT GATE===>'
        self.aicontrol = AIControl()

        # wait for vision service and declare variables for store data from vision
        print '---wait for vision service---'
        rospy.wait_for_service('vision_gate')
        self.gate_req = rospy.ServiceProxy('vision_gate', vision_srv_gate)
        self.gate_data = vision_gate()

    def detectGate(self):
        # store all data from vision service into gate_data
        self.gate_data = self.gate_req(String('gate'), String('gate'))
        self.gate_data = self.gate_data.data

    def run(self):
        # declare auv for shorter variable name
        auv = self.aicontrol

        print '<===Doing Gate===>'
        # move auv to target depth
        auv.depthAbs(cons.GATE_DEPTH)

        # declare all variables used for mission
        count = 0 # counter for mode changing
        reset = 0 # counter for reset count
        back = 0 # counter for moving back in case of auv is too close to the gate
        forward = 0 # counter for moving forward in case of auv is too far from the gate
        center = 0
        mode = 0

if __name__=='__main__':
    rospy.init_node('gate_node')
