#/usr/bin/python2.7

import rospy, math
import constants as cons
import numpy as np
import pickle, sklearn
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
        self.data = self.marker_req(String('marker'))
        self.data = self.data.data

    def run(self):
        auv = self.aicontrol
        mode = 0
        count = 0
        reset = 0

        while not rospy.is_shutdown() and mode != -1:
            self.detectGate()
            found_gate = self.data.appear

            if mode == 0:
                auv.multiMove([0, 0, 0, 0, 0, 0.2])
                if not found_gate:
                    reset += 1
                else:
                    count += 1
                    reset = 0

                if count >= 5:
                    mode = 1

                if reset >= 5:
                    count = 0
                    reset = 0

            if mode == 1:
                auv.multiMove([0, 0, 0, 0, 0, 0.2])
                if found_gate:
                    reset += 1
                else:
                    count += 1
                    reset = 0

                if count >= 5:
                    mode = -1

                if reset >= 5:
                    count = 0
                    reset = 0


if __name__=='__main__':
    rospy.init_node('gate_node')
    gate = Gate()
    gate.run()
