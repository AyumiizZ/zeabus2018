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
        self.aicontrol = AIControl()

        # wait for vision service and declare variables for store data from vision
        print '---wait for vision service---'
        rospy.wait_for_service('vision_casino_gate')
        self.gate_req = rospy.ServiceProxy('vision_casino_gate', vision_srv_casino_gate)
        self.gate_data = vision_casino_gate()

    def detectGate(self):
        # store all data from vision service into gate_data
        self.gate_data = self.gate_req(String('gate'))
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
            cx1 = self.gate_data.cx1
            cx2 = self.gate_data.cx2

            # find gate
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
                    print '<<<Change to mode 1>>>'
                    mode = 1
                elif reset >= 5:
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)

            # move to center of the gate
            if mode == 1:
                print
                print '---mode 1---'
                print 'POS: %d'%(pos)
                print 'cx1: %f'%(cx1)
                print 'cx2: %f'%(cx2)
                print 'area: %f'%(area)
                print '-------------------'
                '''
                # found only left part
                if pos == -1:
                    auv.move('right', cons.AUV_M_SPEED)

                # found only right part
                elif pos == 1:
                    auv.move('left', cons.AUV_M_SPEED)

                # found both left and right
                elif pos == 0:
                '''
                if cx1 < 0:
                    auv.move('left', cons.AUV_M_SPEED*abs(cx1))
                elif cx1 > 0:
                    auv.move('right', cons.AUV_M_SPEED*abs(cx1))

                # check if gate is center or not
                if -cons.VISION_GATE_ERROR <= cx1 <= cons.VISION_GATE_ERROR:
                    print '<<<CENTER>>>'
                    center += 1
                    auv.stop()
                elif -cons.VISION_GATE_ERROR > cx1 > cons.VISION_GATE_ERROR:
                    reset += 1

            # check center's counter
                if center >= 3:
                    print '<<<Change to mode 2>>>'
                    mode = 2
                elif reset >= 10:
                    center = 0
                    reset = 0

                # check if auv is too close or too far from the gate
                if area > 0.01:
                    print 'TOO CLOSE'
                    back += 1
                    forward = 0
                    auv.stop()
                elif 0 < area < 0.005:
                    print 'TOO FAR'
                    forward += 1
                    back = 0
                    auv.stop()
                elif 0.005 <= area <= 0.01:
                    forward = 0
                    back = 0

                # check distance's counter
                if forward >= 3:
                    print 'MOVE CLOSER TO THE GATE'
                    auv.driveX(0.3)
                elif back >= 3:
                    print 'MOVE FURTHUR FROM THE GATE'
                    auv.driveX(-0.4)

            # go through the gate
            if mode == 2:
                print '---mode 2---'
                auv.driveX(8)
                mode = -1

        # passed through gate
        auv.stop()
        print 'Gate completed'

if __name__=='__main__':
    rospy.init_node('gate_node', anonymous=False)
    gate = Gate()
    gate.run()
