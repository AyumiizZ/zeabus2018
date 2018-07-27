#!/usr/bin/python2.7

import rospy
import constants as cons
from aicontrol import AIControl
from zeabus_vision.srv import vision_srv_qualifying_marker
from zeabus_vision.msg import vision_qualifying_marker
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

    def run(self):
        auv = self.aicontrol

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

            if mode == 0:
                print '---mode 0---'

                if appear:
                    count += 1
                    reset = 0
                    print 'FOUND MARKER: %d'%(count)
                    if cx < 0:
                        side = 1
                    elif cx > 0:
                        side = -1
                elif not appear:
                    reset += 1
                    print 'NOT FOUND MARKER: %d'%(reset)
                       # auv.move('', 0, side*0.1)

                if count >= 5:
                    count = 0
                    reset = 0
                    print '<<<Change to mode 1>>>'
                    mode = 1
                    yaw_check = auv.auv_state[5]
                elif reset >= 5:
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)

            if mode == 1:
                if appear:
                    if cx <= -cons.VISION_MARKER_ERROR:
                        auv.move('left', cons.AUV_H_SPEED)
                        side = -1
                    elif cx >= cons.VISION_MARKER_ERROR:
                        auv.move('right', cons.AUV_H_SPEED)
                        side = 1

                    else:

                        if area <= 0.03:
                            print 'too far'
                            auv.move('forward', cons.AUV_L_SPEED)
                        elif area >= 0.09:
                            print 'too close'
                            auv.move('backward', cons.AUV_L_SPEED)
                        if 0.025 < area < 0.4:
                            auv.move('left', cons.AUV_H_SPEED, -cons.AUV_TURN_SPEED)

                    '''
                    if start_rotate and abs(auv.auv_state[5]-yaw_check < 0.5):
                        print '<<<<DONE ROTATE>>>>'
                        auv.stop()
                        mode = 2
                    '''
                else:
                    auv.move('', 0, side*0.1)

            if mode == 2:
                auv.turnRelative(90)
                print 'DONE'
                mode = -1

if __name__=='__main__':
    rospy.init_node('marker_node')
    marker = Marker()
    marker.run()
