#!/usr/bin/python2.7

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_vision.srv import vision_srv_path
from zeabus_vision.msg import vision_path
from aicontrol import AIControl
import constants as cons

class Path(object) :

    def __init__(self) :
        self.data = vision_path()
        print '<===INIT PATH===>'
        self.aicontrol = AIControl()
        rospy.wait_for_service('vision_path')
        self.detect_path = rospy.ServiceProxy('vision_path', vision_srv_path)
        self.centerx = 0
        self.resetx = 0
        self.failed = False

    def detectPath(self) :
        self.data = self.detect_path(String('path'))
        self.data = self.data.data
        # print self.data
        # print 'check data'

    def checkCenter(self) :
        print 'checking'
        auv = self.aicontrol

        is_center = False
        if self.data.cx < 0:
            side = 1
        else:
            side = -1

        data = vision_path()

        while not rospy.is_shutdown() and not is_center:
            data = self.detect_path(String('path'))
            data = data.data
            appear = data.appear
            cx = data.cx
            cy = data.cy
            angle = data.degrees
            ###############################
            print '<---FUCK--->'
            print 'cx: %f'%(cx)
            print 'cy: %f'%(cy)
            print 'appear: %s'%(appear)
            print 'angle: %f'%(angle)
            print '---------------------'
            ###############################
            rospy.sleep(0.4)

            if appear:
                #check cx's center
                if cx < -cons.VISION_PATH_ERROR:
                    auv.move('left', cons.AUV_M_SPEED*(abs(cx)))
                    side = 0.5
                elif cx > cons.VISION_PATH_ERROR:
                    auv.move('right', cons.AUV_M_SPEED*(abs(cx)))
                    side = -0.5

                #check if auv is centerx of path or not
                if -cons.VISION_PATH_ERROR <= cx <= cons.VISION_PATH_ERROR:
                    print '<<<CENTERX: %d>>>'%(self.centerx)
                    self.centerx += 1

                elif -cons.VISION_PATH_ERROR > cx > cons.VISION_PATH_ERROR:
                    print '<<<NOTCENTER: %d>>>'%(self.resetx)
                    self.resetx += 1
            else:
                auv.multiMove([0, side, 0, 0, 0, 0])

            #check centerx's counter
            if self.centerx >= 3:
                self.centerx = 0
                is_center = True
            elif self.resetx >= 10:
                self.centerx = 0
                self.resetx = 0

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

    def isCenter(self):
        if abs(self.data.cx) <= abs(cons.VISION_PATH_ERROR):
            return True
        else:
            return False

    def run(self) :
            auv = self.aicontrol
            checkpoint_x = auv.auv_state[0]
            checkpoint_y = auv.auv_state[1]
            print 'checkpoint_x: %f'%(checkpoint_x)
            print 'checkpoint_y: %f'%(checkpoint_y)
            auv.depthAbs(-1)

            print '<===DOING PATH===>'

            mode = 0
            not_found = 0
            reset_turn = 0
            count_turn = 0
            count = 0
            reset = 0
            sidex = 0
            sidey = 0

            retry = 0
            Pass = False
            while not rospy.is_shutdown() and not mode == -1:
                #find path
                self.detectPath()
                cx = self.data.cx
                cy = self.data.cy
                area = self.data.area
                angle = self.data.degrees
                appear = self.data.appear

                ###############################
                print '<---MODE 1--->'
                print 'cx: %f'%(cx)
                print 'cy: %f'%(cy)
                print 'appear: %s'%(appear)
                print 'angle: %f'%(angle)
                print '---------------------'
                ###############################
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
                        #auv.driveX(0.3)
                        #auv.depthAbs(cons.PATH_DEPTH)
                        self.checkCenter()
                        auv.depthAbs(-2, 0.5)
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
                    auv.move('forward', cons.AUV_M_SPEED)

                if mode == -99:
                    print 'Failed'
                    self.failed = True
                    mode = -1

                if mode == 99:
                    auv.fixXY(checkpoint_x, checkpoint_y)

                    if retry >= 2:
                        mode = -99

                    for _ in range(20):
                        self.detectPath()
                        appear = self.data.appear
                        cx = self.data.cx
                        cy = self.data.cy
                        if appear:
                            auv.move('right', cons.AUV_M_SPEED*cx)
                            count += 1
                            print 'Found path: %d'%(count)
                        else:
                            auv.move('right', cons.AUV_M_SPEED)
                            reset += 1

                        if reset >= 5:
                            count = 0
                            reset = 0
                        if count >= 10:
                            mode = 1
                            count = 0
                            reset = 0
                            self.checkCenter()
                            auv.depthAbs(-2)
                            break
                    if count >= 10:
                        continue

                    for _ in range(20):
                        self.detectPath()
                        appear = self.data.appear
                        cx = self.data.cx
                        cy = self.data.cy
                        if appear:
                            auv.move('forward', cons.AUV_M_SPEED*cy)
                            count += 1
                            print 'Found path: %d'%(count)
                        else:
                            auv.move('forward', cons.AUV_M_SPEED)
                            reset += 1

                        if reset >= 5:
                            count = 0
                            reset = 0
                        if count >= 10:
                            mode = 1
                            count = 0
                            reset = 0
                            self.checkCenter()
                            auv.depthAbs(-2)
                            break
                    if count >= 10:
                        continue

                    for _ in range(20):
                        self.detectPath()
                        appear = self.data.appear
                        cx = self.data.cx
                        cy = self.data.cy
                        if appear:
                            auv.move('left', cons.AUV_M_SPEED*cx*(-1))
                            count += 1
                            print 'Found path: %d'%(count)
                        else:
                            auv.move('left', cons.AUV_M_SPEED)
                            reset += 1

                        if reset >= 5:
                            count = 0
                            reset = 0
                        if count >= 10:
                            mode = 1
                            count = 0
                            reset = 0
                            self.checkCenter()
                            auv.depthAbs(-2)
                            break
                    if count >= 10:
                        continue

                    for _ in range(20):
                        self.detectPath()
                        appear = self.data.appear
                        cx = self.data.cx
                        cy = self.data.cy
                        if appear:
                            auv.move('backward', cons.AUV_M_SPEED*cy*(-1))
                            count += 1
                            print 'Found path: %d'%(count)
                        else:
                            auv.move('backward', cons.AUV_M_SPEED)
                            reset += 1

                        if reset >= 5:
                            count = 0
                            reset = 0
                        if count >= 10:
                            mode = 1
                            count = 0
                            reset = 0
                            self.checkCenter()
                            auv.depthAbs(-2)
                            break
                    if count >= 10:
                        continue

                    retry += 1

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
                        if self.isCenter() :
                            print 'I\'m going on path'
                            auv.move('forward', cons.AUV_L_SPEED)
                        else:
                            self.checkCenter()

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
                            elif sidex == 0:
                                mode = 99
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
                    auv.driveX(1)
                    #auv.move('forward', 3)
                    mode = -1

            # passed through path
            auv.stop()
            print 'Path completed'

if __name__=='__main__':
    rospy.init_node('path_node')
    path = Path()
    path.run()
