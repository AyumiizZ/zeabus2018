#!/usr/bin/python2.7

# Input 'rb'  Big red hole
# Input 'rs'  Small red hole
# Input 'y'   Yellow hole
# Input 'h'   Handle

import rospy
import constants as cons
from aicontrol import AIControl
from zeabus_vision.srv import vision_srv_slots
from zeabus_vision.msg import vision_slots
from std_msgs.msg import String

class Gate(object):
    def __init__(self):
        print '<===INIT GATE===>'
        print '---wait for vision service---'
        rospy.wait_for_service('vision_slots')
        self.gate_req = rospy.ServiceProxy('vision_slots', vision_srv_slots)
        self.gate_data = vision_slots()

    def detectGate(self, task, req):
        try:
            self.gate_data = self.gate_req(String(task), String(req))
            self.gate_data = self.gate_data.data
        except: print 'Error'

    def run(self):
        name = raw_input('Input: ').lower()
        while not rospy.is_shutdown():
            if name == 'rb':
                self.detectGate('red_hole', 'big')
                print '==========================='
                print 'RED HOLE BIG'
                print 'cx: %f'%(self.gate_data.cx)
                print 'cy: %f'%(self.gate_data.cy)
                print 'AREA: %f'%(self.gate_data.area)
                print 'APPEAR: %s'%(self.gate_data.appear)
                print 'MODE: %d'%(self.gate_data.mode)
                print '==========================='

            elif name == 'rs':
                self.detectGate('red_hole', 'small')
                print '==========================='
                print 'RED HOLD SMALL'
                print 'cx: %f'%(self.gate_data.cx)
                print 'cy: %f'%(self.gate_data.cy)
                print 'AREA: %f'%(self.gate_data.area)
                print 'APPEAR: %s'%(self.gate_data.appear)
                print 'MODE: %d'%(self.gate_data.mode)
                print '==========================='

            elif name == 'y':
                self.detectGate('yellow_hole', '')
                print '==========================='
                print 'YELLOW HOLE'
                print 'cx: %f'%(self.gate_data.cx)
                print 'cy: %f'%(self.gate_data.cy)
                print 'AREA: %f'%(self.gate_data.area)
                print 'APPEAR: %s'%(self.gate_data.appear)
                print 'MODE: %d'%(self.gate_data.mode)
                print '==========================='

            elif name == 'h':
                self.detectGate('handle', 'front')
                print '==========================='
                print 'HANDLE'
                print 'cx: %f'%(self.gate_data.cx)
                print 'cy: %f'%(self.gate_data.cy)
                print 'AREA: %f'%(self.gate_data.area)
                print 'APPEAR: %s'%(self.gate_data.appear)
                print 'MODE: %d'%(self.gate_data.mode)
                print '==========================='

            print
            print

if __name__=='__main__':
    rospy.init_node('gate_node', anonymous=False)
    gate = Gate()
    gate.run()
