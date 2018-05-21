#!/usr/bin/python2.7

import rospy
import math as m
import constants as cons
from aicontrol import AIControl
from zeabus_vision.msg import vision_roulette
from zeabus_vision.srv import vision_srv_roulette
from std_msgs.msg import String

class roulette(object) :
    def __init__(self) :
        print '<===INIT BIN===>'
        self.aicontrol = AIControl()
        self.data = vision_roulette
        rospy.wait_for_service('vision_roulette')
        self.detect_roulette = rospy.ServiceProxy('vision_roulette',vision_srv_roulette)

    def detectBin(self) :
        self.data = self.detect_roulette(String('roulette'))
        self.data = self.data.data

    def pinger(self) :
        self.data = self.pinger(String('roulette'))
        self.data = self.data.data
    def checkCenter(self) :
        x = False
        y = False
        centerx = 0
        centery = 0
        resetx = 0
        resety = 0
        auv = self.aicontrol
        cx = self.data.cx
        cy = self.data.cy
        self.detectBin()

        #check cx's center
        if cx > 0:
            auv.move('left', cons.AUV_M_SPEED*abs(cx))
        elif cx < 0:
            auv.move('right', cons.AUV_M_SPEED*abs(cx))


        #check if auv is centerx of bin or not

        if -cons.VISION_ROULETTE_ERROR <= cx <= cons.VISION_ROULETTE_ERROR:
            print '<<<CENTERX>>>'
            centerx += 1
        elif -cons.VISION_ROULETTE_ERROR > cx > cons.VISION_ROULETTE_ERROR:
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

        #check if auv is centery of bin or not
        if -cons.VISION_ROULETTE_ERROR <= cy <= cons.VISION_ROULETTE_ERROR:
            print '<<<CENTERY>>>'
            centery += 1
        elif -cons.VISION_ROULETTE_ERROR > cy > cons.VISION_ROULETTE_ERROR:
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

        print '<===DOING BIN===>'

        mode = 0
        count = 0
        reset = 0 
        while not rospy.is_shutdown() and not mode == -1:

            if mode == 0 : #find pinger
                print '<---mode 0--->'
                self.pinger()

                anglex = self.data.anglex
                angley = self.data.angley
                I = self.data.intensity
                P = self.data.power
                auv.ternRelative(anglex)
                auv.depthRelative(angley)
                auv.driveX(m.sqrt(P/4*m.pi*I))
                auv.stop()
                mode = 1

            if mode == 1 : #find green bin
                print '<---mode 1--->'
                auv.depthAbs(-4, 0.5)
                self.detectBin()
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
                    mode = 2
                elif reset >= 5:
                    reset = 0
                    count = 0
                auv.move('forward', cons.AUV_M_SPEED)

            if mode == 2 : #check bin
                print '<---mode 2--->'
                #############################
                self.detectBin()
                appear = self.data.appear
                cx = self.data.cx
                cy = self.data.cy
                area = self.data.area
                print '--------------------'
                print 'cx:%f'%(cx)
                print 'cy:%f'%(cy)
                print 'appear: %s'%(appear)
                #############################
                if appear :
                    if self.checkCenter :
                        if area >= 1 :
                            print 'let\'s it go!!!'
                            auv.stop()
                            mode = -1
                        elif area < 1 :
                            auv.depthAbs(-0.5, 0.1)
                elif not appear :
                    auv.move('left', con.AUV_L_SPEED)
        print 'Roulette completed'

if__name__=='__main__' :
    rospy.init_node('roulette_node')
    roulette = roulette()
    roulette.run()
