#!/usr/bin/python2.7

import rospy, math
import constants as cons
from aicontrol import AIControl
from zeabus_vision.srv import vision_srv_qualifying_marker
from zeabus_vision.msg import vision_qualifying_marker
#from zeabus_imaging_sonar.msg import sonar_msg
#from zeabus_imaging_sonar.srv import sonar_srv
from std_msgs.msg import String

class Gate(object):
    def __init__(self):
        print '<===INIT MARKER===>'
        self.aicontrol = AIControl()

        print '---wait for vision service---'
        rospy.wait_for_service('vision_qualifying_marker')
        self.marker_req = rospy.ServiceProxy('vision_qualifying_marker', vision_srv_qualifying_marker)
        self.data = vision_qualifying_marker()

    def detectGate(self):
        self.data = self.marker_req(String('gate'))
        self.data = self.data.data

    def run(self):
        auv = self.aicontrol
        mode = 0
        count = 0
        reset = 0

        while not rospy.is_shutdown() and mode != -1:
            self.detectGate()
            found_gate = self.data.appear
            cx = (self.data.cx_right+self.data.cx_left)/2

            if mode == 0:
                print 'mode 0'
                if not found_gate:
                    auv.multiMove([0, 0, 0, 0, 0, 0.1])
                    reset += 1
                    print 'not found gate'
                else:
                    print 'found gate'
                    count += 1
                    reset = 0
                    auv.multiMove([0, 0, 0, 0, 0, 0.05])

                if count >= 5:
                    mode = 1
                    auv.stop()
                    count = 0
                    reset = 0

                if reset >= 5:
                    count = 0
                    reset = 0

            if mode == 1:
                print 'mode 1'
                if found_gate:
                    auv.multiMove([0, 0, 0, 0, 0, 0.05])
                    print 'found gate'
                    reset += 1
                else:
                    auv.stop()
                    print 'not found gate'
                    count += 1
                    reset = 0

                if count >= 5:
                    mode = -1

                if reset >= 5:
                    count = 0
                    reset = 0


if __name__=='__main__':
    rospy.init_node('gate_node')
    print 'init'
    gate = Gate()
    print 'create object'
    gate.run()
