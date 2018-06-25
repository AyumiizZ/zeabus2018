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
        self.center = 0
        self.reset = 0

    def detectBin(self, req) :
        self.data = self.detect_roulette(String('roulette'), String('green'))
        self.data = self.data.data

    def pinger(self) :
        self.data = self.pinger(String('roulette'))
        self.data = self.data.data

    def checkCenter(self) :
        print 'checking'
        auv = self.aicontrol
        cx = self.data.cx
        cy = self.data.cy
        appear = self.data.appear
        self.detectBin('green')

        #check bin center
        if appear :
            if -cons.VISION_ROULETTE_ERROR <= cx <= cons.VISION_ROULETTE_ERROR and -cons.VISION_ROULETTE_ERROR <= cy <= cons.VISION_ROULETTE_ERROR :
                self.center += 1
                print 'center:%d'%center
            else :
                self.reset +=1
                auv.multiMove([cx, cy, 0, 0, 0, 0])


        #check center counter
        if center >= 1:
            print '<<<CHECK CENTER>>>'
            return True
        elif resetx >= 5:
            centerx = 0
            resetx = 0

    def run(self) :
        auv = self.aicontrol

        print '<===DOING BIN===>'

        mode = 1
        count = 0
        reset = 0
        while not rospy.is_shutdown() and not mode == -1:
            '''
            if mode == 0 : #find pinger
                print '<---mode 0--->'
                self.pinger()

                anglex = self.data.anglex
                angley = self.data.angley
                I = self.data.intensity
                auv.ternRelative(anglex)
                if not angley == m.pi/2 :
                    auv.move('forward', cons.AUV_H_SPEED)
                    auv.stop()
                    mode = 1
            '''
            if mode == 1 : #find green bin
                print '<---mode 1--->'
                auv.depthAbs(-2.5, 0.5)
                self.detectBin('green')
                appear = self.data.appear 
                if appear :
                    count += 1
                    reset = 0
                    print 'FOUND BIN: %d'%(count)
                elif not appear:
                    reset += 1
                    print 'NOT FOUND BIN: %d'%(reset)

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
                self.detectBin('green')
                appear = self.data.appear
                cx = self.data.cx
                cy = self.data.cy
                area = self.data.area
                print '--------------------'
                print 'cx:%f'%(cx)
                print 'cy:%f'%(cy)
                print 'appear: %s'%(appear)
                print
                #############################
                if appear :
                    if self.checkCenter() :
                        if area >= 0.03 :
                            print 'let\'s it go!!!'
                            auv.stop()
                            mode = -1
                        elif area < 0.03 :
                            #auv.move('down', cons.AUV_M_SPEED)
                            auv.depthRelative(-0.5, 0.2)
                            auv.stop()
                elif not appear :
                    #auv.move('down', cons.AUV_M_SPEED)
                    auv.depthRelative(0.1, 0.1)
                    auv.stop()
        print 'Roulette completed'

if __name__=='__main__' :
    rospy.init_node('roulette_node')
    roulette = roulette()
    roulette.run()
