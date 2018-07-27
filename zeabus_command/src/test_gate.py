#!/usr/bin/python2.7

import rospy
import constants as cons
from aicontrol import AIControl
from zeabus_vision.srv import vision_srv_casino_gate
from zeabus_vision.msg import vision_casino_gate
from std_msgs.msg import String

class Gate(object):
    def __init__(self):
        print '<===INIT GATE===>'
        print '---wait for vision service---'
        rospy.wait_for_service('vision_casino_gate')
        self.gate_req = rospy.ServiceProxy('vision_casino_gate', vision_srv_casino_gate)
        self.gate_data = vision_casino_gate()

    def detectGate(self):
        # store all data from vision service into gate_data
        try:
            self.gate_data = self.gate_req(String('casino_gate'))
            self.gate_data = self.gate_data.data
        except:print 'error'

    def run(self):
        # declare auv for shorter variable name
        while not rospy.is_shutdown():
            self.detectGate()
            print 'CX1: %f'%(self.gate_data.cx1)
            print 'CX2: %f'%(self.gate_data.cx2)
            print 'AREA: %f'%(self.gate_data.area)
            print 'APPEAR: %s'%(self.gate_data.appear)
            rospy.sleep(0.3)

if __name__=='__main__':
    rospy.init_node('gate_node', anonymous=False)
    gate = Gate()
    gate.run()
