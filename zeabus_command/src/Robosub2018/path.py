#!/usr/bin/python2.7

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_example.srv import vision_srv_path
from zeabus_example.msg import vision_path
from aicontrol import AIControl
import constants as cons

class Path(object) :

    def __init__(self) :
        self.data = vision_path
        print '<===INIT PATH===>'
        self.aicontrol = AIcontrol()
        rospy.wait_for_service('vision_path')
        self.detect_path = rospy.ServiceProxy('vision_path', vision_srv_path)


    def detectPath(self, piece) :
        self.data = self.detect_path(String('path'), String(str(piece)))
        self.data = self.data.data


    def checkCenter(self) :
        x = False
        y = False
        centerx = 0
        centery = 0
        resetx = 0
        resety = 0
        cx = self.data.cx
        cy = seif.data.cy
        self.detectPath()

        #check cx's center
        if cx < 0:
            auv.move('left', cons.AUV_M_SPEED*abs(cx))
        elif cx > 0:
            auv.move('right', cons.AUV_M_SPEED*abs(cx))

        #check if auv is centerx of path or not
        if -cons.VISION_PATH_ERROR <= cx <= cons.VISION_PATH_ERROR:
            print '<<<CENTERX>>>'
            centerx += 1
        elif -cons.VISION_PATH_ERROR > cx > cons.VISION_PATH_ERROR:
            resetx += 1

        #check centerx's counter
        if centerx >= 3:
            x = True
        elif resetx >= 10:
            centerx = 0
            resetx = 0

        #check cy's center
        if cy < 0:
            auv.move('forward', cons.AUV_M_SPEED*abs(cy))
        elif cy > 0:
            auv.move('backward', cons.AUV_M_SPEED*abs(cy))

        #check if auv is centery of path or not
        if -cons.VISION_PATH_ERROR <= cy <= cons.VISION_PATH_ERROR:
            print '<<<CENTERY>>>'
            centery += 1
        elif -cons.VISION_PATH_ERROR > cy > cons.VISION_PATH_ERROR:
            resety += 1

        #check center's counter
        if centery >= 3:
            y = True
        elif resety >= 10:
            centery = 0
            resety = 0
        if x and y :
            print '<<<CENTER>>>'
            return True
        else :
            return False


    def run(self) :
        auv = self.aicontrol

        print '<===DOING PATH===>'

        auv.depthAbs(cons.PATH_DEPTH)

        mode = 0
        count = 0
        reset = 0
        while not rospy.is_shutdown() and not mode == -1:
            #find path
            if mode == 0 :
                print '<---MODE 0--->'
                self.detectPath('edge')
                appear = self.data.appear
                #check if path appear
                if appear :
                    count += 1
                    reset = 0
                    print 'FOUND PATH: %d'%(count)
                elif not appear:
                    reset += 1
                    print 'NOT FOUND PATH: %d'%(reset)

                # check counter
                if count >= 5:
                    count = 0
                    reset = 0
                    print 'let\' to adjust->>>'
                    auv.stop()
                    mode = 1
                elif reset >= 5:
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)
            #adjust angle and center before go on path
            if mode == 1 :
                print '<---MODE 1--->'
                print 'cx: %f'%(cx)
                print 'cy: %f'%(cy)
                self.detechPath('edge')
                if self.checkCenter() :
                    auv.turnRel(angle)
                    print 'I\'m ready!!'
                    auv.stop()
                    mode = 2

            #go on path to angle
            if mode == 2 :
                print '<---MODE 2--->'
                print 'cx: %f'%(cx)
                print 'cy: %f'%(cy)
                angle = self.data.angle
                req_appear = self.data.req_appear
                self.detechPath('angle')
                #if find angle of path
                if self.checkCenter() :
                    if req_appear :
                        self.checkCenter
                        auv.turnRel(angle)
                        print 'Go to finish path :D'
                        mode = 3
                    elif not req_appear :
                        auv.move('forward', cons.AUV_M_SPEED)

            #from angle to finish path
            if mode == 3 :
                print '<---mode 3--->'
                auv.driveX(1.5)
                mode = -1

        # passed through path
        auv.stop()
        print 'Path completed'

if __name__=='__main__':
    rospy.init_node('path_node')


