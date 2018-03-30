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
    self.data = self.detect_path(String('path'))
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
            self.detectPath()
            angle = self.data.angle
            if mode == 0 :
                print '<---MODE 0--->'
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
                    print 'let\'s to adjust->>>'
                    auv.stop()
                    mode = 1
                elif reset >= 5:
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)
            #go on path
            if mode == 1 :
                print '<---MODE 1--->'
                print 'cx: %f'%(cx)
                print 'cy: %f'%(cy)
                if angle >= 15 :
                    auv.turnRelative(angle)
                if checkCenter() :
                    print 'I\'m going on path'
                if appear :
                    reset += 1
                    print 'FOUND PATH: %d'%(count)
                elif not appear:
                    reset = 0
                    count += 1
                    print 'NOT FOUND PATH: %d'%(reset)

                # check counter
                if count >= 5:
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
                mode = -1

        # passed through path
        auv.stop()
        print 'Path completed'

if __name__=='__main__':
    rospy.init_node('path_node')


