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

        while not rospy.is_shutdown() and not mode == -1:
            # get data from vision service and store in temp variables
            self.detectGate()
            area = self.gate_data.area
            appear = self.gate_data.appear
            cx = self.gate_data.cx
            pos = self.data.gate_data.pos

            if mode == 0:
                print '---mode 0---'
                # check if gate is appear
                if appear:
                    count += 1
                    reset = 0
                    print 'FOUND GATE: %d'%(count)
                elif not appear:
                    reset += 1
                    print 'NOT FOUND GATE: %d'%(reset)

                # check counter
                if count >= 5:
                    count = 0
                    reset = 0
                    print '<<<Change to mode 2>>>'
                    mode = 2
                elif reset >= 5:
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)

if __name__=='__main__':
    rospy.init_node('gate_node')
