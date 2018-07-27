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
        print("found frist time")
        self.gate_req = rospy.ServiceProxy('vision_casino_gate', vision_srv_casino_gate)
        self.gate_data = vision_casino_gate()


    def detectGate(self):
        # store all data from vision service into gate_data
        try:
            print("will find service vision_casino_gate")
            rospy.wait_for_service('vision_casino_gate' , 2)
            print("found service vision_casino_gate")
            self.gate_data = self.gate_req(String('casino_gate'))
            self.gate_data = self.gate_data.data
            print("success convert data for use")
        except:
            print("don't found service vision_casino_gate")

    def run(self):
        # declare auv for shorter variable name
        auv = self.aicontrol
        checkpoint_x = auv.auv_state[0]
        checkpoint_y = auv.auv_state[1]

        print '<===Doing Gate===>'
        # move auv to target depth
        auv.depthAbs(cons.GATE_DEPTH)

        auv.turnAbs(cons.GATE_DEGREES)
        auv.driveX(5)

        # declare all variables used for mission
        count = 0 # counter for mode changing
        reset = 0 # counter for reset count
        back = 0 # counter for moving back in case of auv is too close to the gate
        forward = 0 # counter for moving forward in case of auv is too far from the gate
        center = 0
        mode = 0

        side = -1
        prev_ratio = 0
        count_ratio = 0
        reset_ratio = 0

        count_round = 0
        count_not_found = 0
        changed = False

        have_found = False

        while not rospy.is_shutdown() and not mode == -1:
            # get data from vision service and store in temp variables
            self.detectGate()
            area = self.gate_data.area
            appear = self.gate_data.appear
            cx1 = self.gate_data.cx1
            cx2 = self.gate_data.cx2
            ratio = self.gate_data.w_h_ratio

            # find gate
            if mode == 0:
                print '---mode 0---'

                print '==============='
                print 'count round: %d'%(count_round)
                print '==============='
                if count_round >= 50:
                    temp_x = auv.auv_state[0]
                    temp_y = auv.auv_state[1]
                    count_round = 0
                    for _ in range(10):
                        auv.multiMove([0, side, 0, 0, 0, 0])

                    rospy.sleep(5)
                    side *= -1

                # check if gate is appear
                if appear:
                    have_found = True
                    if abs(ratio - 2) <= 0.3:
                        count += 1
                    reset = 0
                    print 'FOUND GATE: %d'%(count)

                    if abs(ratio - 2) < abs(prev_ratio - 2):
                        count_ratio += 1
                        reset_ratio = 0
                    else:
                        reset_ratio += 1

                    if count_ratio >= 4:
                        count_not_found = 0
                        count_ratio = 0
                        reset_ratio = 0
                        side *= -1
                    elif reset_ratio >= 5:
                        count_ratio = 0
                        reset_ratio = 0

                    print '========================'
                    print '========================'
                    print
                    print 'Count ratio: %d'%(count_ratio)
                    print 'side: %d'%(side)
                    print 'ratio: %f'%(ratio)
                    print
                    print '========================'
                    print '========================'

                    prev_ratio = ratio

                elif not appear:
                    if have_found:
                        auv.multiMove([0, 0, 0, 0, 0, side*0.01 * (abs(ratio - 2))])
                    else:
                        auv.multiMove([0, 0, 0, 0, 0, -0.01])
                    reset += 1
                    print 'NOT FOUND GATE: %d'%(reset)

                # check counter
                if count >= 8:
                    count = 0
                    reset = 0
                    print '<<<Change to mode 1>>>'
                    mode = 1
                elif reset >= 5:
                    reset = 0
                    count = 0

                count_round += 1
                rospy.sleep(0.1)

            # move to center of the gate
            if mode == 1:
                print
                print '---mode 1---'
                print 'cx1: %f'%(cx1)
                print 'cx2: %f'%(cx2)
                print 'area: %f'%(area)
                print '-------------------'
                # check if auv is too close or too far from the gate
                if area > 0.2:
                    print 'TOO CLOSE'
                    #for _ in range(10):
                    auv.move('backward', cons.AUV_M_SPEED)

                elif 0 < area < 0.16:
                    print 'TOO FAR'
                    #for i in range(15):
                    auv.move('forward', cons.AUV_M_SPEED)

                if appear:
                    if cx1 < 0:
                        #for i in range(10):
                        auv.move('left', cons.AUV_H_SPEED*abs(cx1))
                    elif cx1 > 0:
                        #for i in range(10):
                        auv.move('right', cons.AUV_H_SPEED*abs(cx1))

                else:
                    auv.move('right', cons.AUV_M_SPEED)

                # check if gate is center or not
                if -cons.VISION_GATE_ERROR <= cx1 <= cons.VISION_GATE_ERROR:
                    print '<<<CENTER>>>'
                    center += 1
                    auv.stop()
                elif -cons.VISION_GATE_ERROR > cx1 > cons.VISION_GATE_ERROR:
                    reset += 1

            # check center's counter
                if center >= 8 and area > 0.16:
                    print '<<<Change to mode 2>>>'
                    mode = 2
                elif reset >= 3:
                    center = 0
                    reset = 0

            # go through the gate
            if mode == 2:
                print '---mode 2---'
                auv.driveX(7)
                mode = -1

            rospy.sleep(0.2)

        # passed through gate
        auv.stop()
        print 'Gate completed'

if __name__=='__main__':
    rospy.init_node('gate_node', anonymous=False)
    gate = Gate()
    gate.run()
