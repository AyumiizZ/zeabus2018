#!/usr/bin/python2.7
import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_vision.srv import path_srv
from zeabus_vision.msg import path

class Path(object) :

    def __init__(self) :
        self.data = path_srv
        print '<===INIT PATH===>'
        rospy.wait_for_service('vision_path')
        self.detect_path = rospy.ServiceProxy('vision_path', path_srv)

    def detectPath(self) :
        self.data = self.detect_path(String('path'))
        self.data = self.data.data

    def checkCenter(self) :
        print 'checking'
        x = False
        y = False
        centerx = 0
        centery = 0
        resetx = 0
        resety = 0
        cx = self.data.cx
        cy = self.data.cy
        self.detectPath()

        #check cx's center
        if cx > 0:
            print 'move left speed 1.5*%f'%(cx)
        elif cx < 0:
            print 'move right speed 1.5*%f'%(cx)

        #check if auv is centerx of path or not
        if -0.3 <= cx <= 0.3:
            print '<<<CENTERX>>>'
            centerx += 1
        elif -0.3 > cx > 0.3:
            resetx += 1

        #check centerx's counter
        if centerx >= 3:
            x = True
        elif resetx >= 10:
            centerx = 0
            resetx = 0

        #check cy's center
        if cy < 0:
            print 'move forward speed 1.5*%f'%(cy)
        elif cy > 0:
            print 'move backward speed 1.5*%f'%(cy)

        #check if auv is centery of path or not
        if -0.3 <= cy <= 0.3:
            print '<<<CENTERY>>>'
            centery += 1
        elif -0.3 > cy > 0.3:
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

        print '<===DOING PATH===>'


        mode = 0
        count = 0
        reset = 0
        sidex = 0
        sidey = 0
        while not rospy.is_shutdown() and not mode == -1:
            #find path
            self.detectPath()
            cx = self.data.cx
            cy = self.data.cy
            appear = self.data.appear
            angle = self.data.degrees
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
                    mode = 1
                elif reset >= 5:
                    reset = 0
                    count = 0
                print 'move forward speed 1.5'
                 #go on path
            if mode == 1 :
                ###############################
                print '<---MODE 1--->'
                print 'cx: %f'%(cx)
                print 'cy: %f'%(cy)
                print 'appear: %s'%(appear)
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

                    if abs(angle) >= 7 :
                        print 'turn degrees %f '%(angle)
                    if self.checkCenter() :
                        print 'I\'m going on path'
                        print 'move forward speed 1.5'
                    reset += 1
                    print 'FOUND PATH: %d'%(count)
                elif not appear:
                    reset = 0
                    count += 1
                    print 'NOT FOUND PATH: %d'%(reset)

                    if sidex > 0 :
                        print 'right'
                    elif sidex < 0 :
                        print 'left'
                    if sidey > 0 :
                        print 'forward'
                    elif sidey < 0 :
                        print 'backward'


                # check counter
                if count >= 10:
                    count = 0
                    reset = 0
                    print 'let\'s to adjust->>>'
                    mode = 2
                elif reset >= 5:
                    reset = 0
                    count = 0
            # exist path yatta!
            if mode == 2 :
                print 'move forward speed 3'
                mode = -1
            rospy.sleep(1)

        # passed through path
        print 'Path completed'



if __name__=='__main__':
    rospy.init_node('path_node')
    path = Path()
    path.run()
