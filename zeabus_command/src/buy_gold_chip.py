#!/usr/bin/python2.7
import rospy
from zeabus_vision.msg import vision_chip
from zeabus_vision.srv import vision_srv_chip
from aicontrol import AIControl
from std_msgs.msg import String, Float64, Bool
import constants as cons
class BuyGoldChip(object) :

    def __init__(self) :
        self.data = vision_chip
        self.aiccontrol = AIControl()

        print '<===INIT BUY_GOLD_CHIP===>'

        rospy.wait_for_service(vision_chip)
        self.detect_chip = rospy.ServiceProxy('vision_chip', vision_srv_chip)
        self.center = 0
        self.reset = 0
    def detectPlate(self) :
        self.data = self.detect_chip(String('plate'))
        self.data = self.data.data
    '''
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
        self.detectChip()

        #check cx's center
        if cx > 0:
            auv.move('left', cons.AUV_M_SPEED*abs(cx))
        elif cx < 0:
            auv.move('right', cons.AUV_M_SPEED*abs(cx))

        #check if auv is centerx of plate or not
        if -cons.VISION_PLATE_ERROR <= cx <= cons.VISION_PLATE_ERROR:
            print '<<<CENTERX>>>'
            centerx += 1
        elif -cons.VISION_PLATE_ERROR > cx > cons.VISION_PLATE_ERROR:
            resetx += 1

        #check centerx's counter
        if centerx >= 3:
            x = True
        elif resetx >= 10:
            centerx = 0
            resetx = 0

        #check cy's center
        if cy < 0:
            auv.move('up', cons.AUV_M_SPEED*abs(cy))
        elif cy > 0:
            auv.move('down', cons.AUV_M_SPEED*abs(cy))

        #check if auv is centery of plate or not
        if -cons.VISION_PLATE_ERROR <= cy <= cons.VISION_PLATE_ERROR:
            print '<<<CENTERY>>>'
            centery += 1
        elif -cons.VISION_PLATE_ERROR > cy > cons.VISION_PLATE_ERROR:
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
    '''
    def checkCenter(self) :

        print 'checking'
        auv = self.aicontrol
        cx = self.data.cx
        cy = self.data.cy
        self.detectChip()
        #check bin center
        if appear :
            if -cons.VISION_ROULETTE_ERROR <= cx <= cons.VISION_ROULETTE_ERROR and -cons.VISION_ROULETTE_ERROR <= cy <= cons.VISION_ROULETTE_ERROR :
                self.center += 1
                print 'center:%d'%center
            else :
                self.reset +=1
                auv.multiMove([0, cx, cy 0, 0, 0])


        #check center counter
        if center >= 1:
            print '<<<CHECK CENTER>>>'
            return True
        elif resetx >= 5:
            centerx = 0
            resetx = 0

    def run(self) :
        auv = self.aicontrol

        print '<===PUSHING PLATE===>'

        auv.depthAbs(cons.PLATE_DEPTH)

        mode = 0
        count = 0
        reset = 0

        while not rospy.is_shutdown() and not maod == -1 :
            #find plate
            self.detectPlate()
            cx = self.data.cx
            cy = self.data.cy
            appear = self.data.appear
            area = self.data.area
            if mode == 0 :
                print '<---mode 0--->'
                if appear :
                    count += 1
                    reset = 0
                    print 'FOUND PLATE: %d'%(count)
                elif not appear:
                    reset += 1
                    print 'NOT FOUND PLATE: %d'%(reset)

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
            #check on center of plate
            if mode == 1 :
                ################################
                print
                print '<---mode 1--->'
                print 'cx: %f'%(cx)
                print 'cy: %f'%(cy)
                print 'appear: %s'%(appear)
                print 'area: %f'%(area)
                print '-----------------------'
                ################################
                if appear :
                    if cx > 0:
                            sidex = 1
                    elif cx < 0:
                        sidex = -1
                    if cy > 0:
                        sidey = 1
                    elif cy < 0:
                         sidey = -1
                    if self.checkCenter() :
                        print 'colliding with plate'
                        auv.move('forward', cons.AUV_M_SPED)
                    if area == 1 :
                        print 'collided'
                        auv.stop()
                    reset += 1
                    print 'FOUND PLATE: %d'%(count)
                elif not appear :
                    reset = 0
                    count += 1
                    print 'NOT FOUND PLATE: %d'%(reset)
                    if sidex > 0 :
                        auv.move('right', cons.AUV_M_SPEED*abs(sidex))
                    elif sidex < 0 :
                        auv.move('left', cons.AUV_M_SPEED*abs(sidex))
                    if sidey > 0 :
                        auv.move('forward', cons.AUV_M_SPEED*abs(sidey))
                    elif sidey < 0 :
                        auv.move('backward', cons.AUV_M_SPEED*abs(sidey))

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
            # find gold chip
            if mode == 2 :
                pass


if __name__ == '__main__' :
    rospy.init_node('buy_gold_chip_node')
    buy_gold_chip = BuyGoldChip()
    BuyGoldChip.run()

