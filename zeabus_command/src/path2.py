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
        self.centerx = 0
        self.resetx = 0

    def detectPath(self) :
        self.data = self.detect_path(String('path'))
        self.data = self.data.data
        # print self.data
        # print 'check data'

    def checkCenter(self) :
        print 'checking'
        auv = self.aicontrol
        cx = self.data.cx
        cy = self.data.cy
        self.detectPath()

        #check cx's center
        if cx < -(cons.VISION_PATH_ERROR-0.1):
            for i in range(3) :
                auv.move('left', cons.AUV_M_SPEED*(abs(cx)+1))
        elif cx > (cons.VISION_PATH_ERROR-0.1):
            for i in range(3) :
                auv.move('right', cons.AUV_M_SPEED*(abs(cx)+1))

        #check if auv is centerx of path or not
        if -cons.VISION_PATH_ERROR <= cx <= cons.VISION_PATH_ERROR:
            print '<<<CENTERX:%d>>>'%(self.centerx)
            self.centerx += 1

        elif -cons.VISION_PATH_ERROR > cx > cons.VISION_PATH_ERROR:
            print '<<<NOTCENTER:%d>>>'%(self.resetx)
            self.resetx += 1

        #check centerx's counter
        if self.centerx >= 3:
            self.centerx = 0
            return True
        elif self.resetx >= 10:
            self.centerx = 0
            self.resetx = 0
        return False

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
        if x  :
            print '<<<CENTER>>>'
            return True
        else :
            return False
        '''



    def run(self) :
            auv = self.aicontrol
            checkpoint_x = auv.auv_state[0]
            checkpoint_y = auv.auv_state[1]

            print '<===DOING PATH===>'

            mode = 0
            not_found = 0
            reset_turn = 0
            count_turn = 0
            count = 0
            reset = 0
            sidex = 0
            sidey = 0
            Pass = False
            while not rospy.is_shutdown() and not mode == -1:
                #find path
                self.detectPath()
                cx = self.data.cx
                cy = self.data.cy
                area = self.data.area
                angle = self.data.degrees
                appear = self.data.appear
                if mode == 0 :
                    print '<---MODE 0--->'
                    #check if path appear
                    if appear and area > 0.05:
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
                        auv.driveX(0.3)
                        #auv.depthAbs(cons.PATH_DEPTH)
                        auv.depthAbs(-2.5, 0.5)
                        mode = 1
                    elif reset >= 5:
                        not_found += 1
                        reset = 0
                        count = 0

                    elif not_found >= 5:
                        print 'Path not found'
                        mode = 99
                        count = 0
                        reset = 0
                    rospy.sleep(0.2)
                    for _ in range(10):
                        auv.move('forward', cons.AUV_M_SPEED)

                if mode == 99:
                    auv.depthAbs(1)
                    auv.fixXY(checkpoint_x, checkpoint_y)
                    for _ in range(20):
                        self.detectPath()
                        appear = self.data.appear
                        cx = self.data.cx
                        cy = self.data.cy
                        if appear:
                            for _ in range(5):
                                auv.move('right', cons.AUV_M_SPEED*cx)
                            count += 1
                            print 'Found path: %d'%(count)
                        else:
                            for _ in range(5):
                                auv.move('right', cons.AUV_H_SPEED)
                            reset += 1

                        if reset >= 5:
                            count = 0
                            reset = 0
                        if count >= 10:
                            mode = 1
                            auv.depthAbs(-2.5)
                            break
                    if count >= 10:
                        continue

                    for _ in range(20):
                        self.detectPath()
                        appear = self.data.appear
                        cx = self.data.cx
                        cy = self.data.cy
                        if appear:
                            for _ in range(5):
                                auv.move('forward', cons.AUV_M_SPEED*cy)
                            count += 1
                            print 'Found path: %d'%(count)
                        else:
                            for _ in range(5):
                                auv.move('forward', cons.AUV_H_SPEED)
                            reset += 1

                        if reset >= 5:
                            count = 0
                            reset = 0
                        if count >= 10:
                            mode = 1
                            auv.depthAbs(-2.5)
                            break
                    if count >= 10:
                        continue

                    for _ in range(20):
                        self.detectPath()
                        appear = self.data.appear
                        cx = self.data.cx
                        cy = self.data.cy
                        if appear:
                            for _ in range(5):
                                auv.move('left', cons.AUV_M_SPEED*cx*(-1))
                            count += 1
                            print 'Found path: %d'%(count)
                        else:
                            for _ in range(5):
                                auv.move('left', cons.AUV_H_SPEED)
                            reset += 1

                        if reset >= 5:
                            count = 0
                            reset = 0
                        if count >= 10:
                            mode = 1
                            auv.depthAbs(-2.5)
                            break
                    if count >= 10:
                        continue

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
                            Pass = False
                        elif cy < 0:
                            sidey = -1
                            Pass = True
                        if abs(angle) >= 15 :
                            count_turn += 1
                        else:
                            reset_turn += 1

                        if count_turn >= 5:
                            count_turn = 0
                            auv.turnRelative(angle, 1)

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
                        if not Pass:
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
