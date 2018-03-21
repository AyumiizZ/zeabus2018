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

if __name__=='__main__':
    rospy.init_node('gate_node')
