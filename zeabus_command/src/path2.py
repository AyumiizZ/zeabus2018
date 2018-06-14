#!/usr/bin/python2.7

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_vision.srv import vision_srv_path
from zeabus_vision.msg import vision_path
from aicontrol import AIControl
import constants as cons

class Path(object) :

    def __init__(self) :
        self.data = vision_path
        print '<===INIT PATH===>'
        self.aicontrol = AIControl()
        rospy.wait_for_service('vision_path')
        self.detect_path = rospy.ServiceProxy('vision_path', vision_srv_path)

    def detectPath(self) :
        self.data = self.detect_path(String('path'))
        self.data = self.data.data
        # print self.data
        # print 'check data'

    def checkCenter(self) :
        print 'checking'
        x = False
        y = False
        centerx = 0
        centery = 0
        resetx = 0
        resety = 0
        auv = self.aicontrol
        cx = self.data.cx
        cy = self.data.cy
        self.detectPath()

        #check cx's center
        if cx < -(cons.VISION_PATH_ERROR-0.1):
            for i in range(3) :
                auv.move('left', cons.AUV_M_SPEED*abs(cx+1))
        elif cx > (cons.VISION_PATH_ERROR-0.1):
            for i in range(3) :
                auv.move('right', cons.AUV_M_SPEED*abs(cx+1))

        #check if auv is centerx of path or not
        if -cons.VISION_PATH_ERROR <= cx <= cons.VISION_PATH_ERROR:
            print '<<<CENTERX>>>'
            centerx += 1
        elif -cons.VISION_PATH_ERROR > cx > cons.VISION_PATH_ERROR:
            resetx += 1

        #check centerx's counter
        if centerx >= 3:
            centerx = 0
            return True
        elif resetx >= 10:
            centerx = 0
            resetx = 0

        #check cy's center
        '''
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
        '''



    def run(self) :
            auv = self.aicontrol

            print '<===DOING PATH===>'

            #auv.depthAbs(cons.PATH_DEPTH)
            auv.depthAbs(-2.5, 0.5)

            mode = 0
            reset_turn = 0
            count_turn = 0
            count = 0
            reset = 0
            sidex = 0
            sidey = 0
            while not rospy.is_shutdown() and not mode == -1:
                #find path
                self.detectPath()
                cx = self.data.cx
                cy = self.data.cy
                angle = self.data.degrees
                appear = self.data.appear
                if mode == 0 :
                    print '<---MODE 0--->'
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
                        print 'let\'s to adjust->>>'
                        auv.stop()
                        auv.driveX(0.5)
                        mode = 1
                    elif reset >= 5:
                        reset = 0
                        count = 0

                    auv.move('forward', cons.AUV_M_SPEED)
                #go on path
                if mode == 1 :
                    ###############################
                    print '<---MODE 1--->'
                    print 'cx: %f'%(cx)
                    print 'cy: %f'%(cy)
                    print 'appear: %s'%(appear)
                    print 'angle: %f'%(angle)
                    print '---------------------'
                    ###############################
                    if appear :
                        if cx > 0:
                            sidex = 1
                        elif cx < 0:
                            sidex = -1
                        if cy > 0:
                            sidey = 1
                        elif cy < 0:
                            sidey = -1
                        if abs(angle) >= 15 :
                            count_turn += 1
                            auv.turnRelative(angle, 1)
                        else:
                            reset_turn += 1

                        if reset_turn >=5:
                            reset_turn = 0
                            count_turn = 0
                        if self.checkCenter() :
                            print 'I\'m going on path'
                            for _ in range(5):
                                auv.move('forward', cons.AUV_L_SPEED)
                        reset += 1
                        print 'FOUND PATH: %d'%(count)
                    elif not appear:
                        reset = 0
                        count += 1
                        print 'NOT FOUND PATH: %d'%(reset)
                        if sidex > 0 :
                            auv.move('right', cons.AUV_M_SPEED)
                        elif sidex < 0 :
                            auv.move('left', cons.AUV_M_SPEED)
                        '''
                        if sidey > 0 :
                            auv.move('forward', cons.AUV_M_SPEED*sidey)
                        elif sidey < 0 :
                            auv.move('backward', cons.AUV_M_SPEED*sidey)
                        '''

                    # check counter
                    if count >= 10:
                        count = 0
                        reset = 0
                        print 'let\'s to adjust->>>'
                        auv.stop()
                        mode = 2
                    elif reset >= 5:
                        reset = 0
                        count = 0
                # exist path yatta!
                if mode == 2 :
                    auv.driveX(2)
                    #auv.move('forward', 3)
                    mode = -1

            # passed through path
            auv.stop()
            print 'Path completed'

if __name__=='__main__':
    rospy.init_node('path_node')
    path = Path()
    path.run()
