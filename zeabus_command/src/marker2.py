#!/usr/bin/python2.7

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

class Marker(object):
    def __init__(self):
        print '<===INIT MARKER===>'
        self.aicontrol = AIControl()

        print '---wait for vision service---'
        rospy.wait_for_service('vision_qualifying_marker')
        self.marker_req = rospy.ServiceProxy('vision_qualifying_marker', vision_srv_qualifying_marker)
        self.data = vision_qualifying_marker()

    def detectMarker(self):
        self.data = self.marker_req(String('marker'))
        self.data = self.data.data

    def linear(self, area):
        return ((-42.46119459)*(area) + 3.76897) +  1

    def run(self):
        auv = self.aicontrol
        #pickle_in = open('linearregression.pickle', 'rb')
        #clf = pickle.load(pickle_in)

        mode = 0
        count = 0
        reset = 0
        yaw_check = 0
        side = 0
        start_rotate = False
        while not rospy.is_shutdown() and not mode == -1:
            self.detectMarker()
            appear = self.data.appear
            area = self.data.area
            cx = (self.data.cx_right + self.data.cx_left)/2
            pred = 0

            if mode == 0:
                print '---mode 0---'

                if appear:
                    print 'AREA: %f'%(area)
                    count += 1
                    pred = area
                    reset = 0
                    print 'FOUND MARKER: %d'%(count)
                    if cx < 0:
                        side = -1
                    elif cx > 0:
                        side = 1
                    #print clf.predict(np.array([[0, pred]]))
                    print self.linear(pred)
                elif not appear:
                    reset += 1
                    print 'NOT FOUND MARKER: %d'%(reset)
                    #auv.move('', 0, side*0.1)

                if count >= 5:
                    print 'pred %f'%pred
                    count = 0
                    reset = 0
                    print '<<<Change to mode 1>>>'
                    mode = 1
                elif reset >= 5:
                    pred = 0
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)
            if mode == 1:
                if cx <= -cons.VISION_MARKER_ERROR:
                    auv.move('left', cons.AUV_M_SPEED)
                elif cx >= cons.VISION_MARKER_ERROR:
                    auv.move('right', cons.AUV_M_SPEED)
                else:
                    mode = 2

            if mode == 2:
                #pred = np.array([[0, pred]])
                print pred
                #dis = clf.predict(pred)
                dis = self.linear(pred)
                print 'Predict: %f'%(dis)
                auv.driveY(-1.7)
                auv.driveX(dis+1.5)
                auv.driveY(3.4)
                auv.driveX(-(dis+1.5))
                auv.driveY(-1.7)
                mode = 3

            if mode == 3:
                auv.turnRelative(182)
                #auv.driveX(1.5+dis+5.5)
                print 'DONE'
                mode = -1

if __name__=='__main__':
    rospy.init_node('marker_node')
    marker = Marker()
    marker.run()
