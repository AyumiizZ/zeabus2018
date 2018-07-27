#!/usr/bin/python2.7
import rospy
from zeabus_vision.msg import vision_buy_a_gold_chip
from zeabus_vision.srv import vision_srv_buy_a_gold_chip
from aicontrol import AIControl
from std_msgs.msg import String, Float64, Bool
import constants as cons
class BuyGoldChip(object) :

    def __init__(self) :
        print '<===INIT BUY_GOLD_CHIP===>'
        self.data = vision_buy_a_gold_chip()
        self.plate = vision_buy_a_gold_chip()
        self.ball = vision_buy_a_gold_chip()
        self.aicontrol = AIControl()

        print 'wait for vision service'
        rospy.wait_for_service('vision_buy_a_gold_chip')
        self.detect_buy_a_gold_chip = rospy.ServiceProxy('vision_buy_a_gold_chip', vision_srv_buy_a_gold_chip)
        self.center = 0
        self.reset = 0

    def detectPlate(self) :
        self.data = self.detect_buy_a_gold_chip(String('buy_a_gold_chip'), String('front'))
        self.plate = self.data.data

    def detectBall(self) :
        self.data = self.detect_buy_a_gold_chip(String('buy_a_gold_chip'), String('bottom'))
        self.ball = self.data.data

    def checkCenter(self) :

        print 'checking'
        auv = self.aicontrol
        cx = self.plate.cx
        cy = self.plate.cy
        appear = self.plate.appear
        self.detectPlate()
        #self.detectBall()
        #check bin center
        if appear :
            if -cons.VISION_PLATE_ERROR <= cx <= cons.VISION_PLATE_ERROR and -cons.VISION_PLATE_ERROR <= cy <= cons.VISION_PLATE_ERROR :
                #self.center += 1
                print 'CENTER'
                auv.stop()
                return True
            elif -cons.VISION_PLATE_ERROR <= cx <= cons.VISION_PLATE_ERROR and not-cons.VISION_PLATE_ERROR <= cy <= cons.VISION_PLATE_ERROR :
                auv.multiMove([0, 0, cy, 0, 0, 0])
            elif not -cons.VISION_PLATE_ERROR <= cx <= cons.VISION_PLATE_ERROR and -cons.VISION_PLATE_ERROR <= cy <= cons.VISION_PLATE_ERROR :
                auv.multiMove([0, cx, 0, 0, 0, 0])
            else:
                self.reset +=1
                auv.multiMove([0, -cx, cy, 0, 0, 0])
                print 'NOT CENTER'
                return False
        else:
            print 'not found plate (check center)'
            return False

        #check center counter
        if self.center >= 3:
            print '<<<CHECK CENTER>>>'
            self.center = 0
            self.reset = 0
            return True
        elif self.reset >= 5:
            self.center = 0
            self.reset = 0

    def run(self) :
        auv = self.aicontrol

        print '<===PUSHING PLATE===>'

        #auv.depthAbs(cons.PLATE_DEPTH)

        mode = 0
        count = 0
        reset = 0
        prev_mode = 0
        side_x = 0
        side_y = 0

        while not rospy.is_shutdown() and not mode == -1 :
            #find plate
            self.detectPlate()
            cx = self.plate.cx
            cy = self.plate.cy
            appear = self.plate.appear
            area = self.plate.area
            hit = self.plate.hit

            if mode == 0 :
                print '<---mode 0--->'
                if appear :
                    auv.stop()
                    count += 1
                    reset = 0
                    print 'FOUND PLATE: %d'%(count)
                    side_x = -cx
                    side_y = cy
                elif not appear:
                    if side_x == 0 and side_y == 0:
                        print 'not found'
                        auv.move('forward', cons.AUV_L_SPEED)
                    else:
                        print 'find plate'
                        auv.multiMove([0, cons.AUV_L_SPEED*side_x, cons.AUV_L_SPEED*side_y, 0, 0, 0])

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

            #check on center of plate
            if mode == 1 :
                ################################
                print
                print '<---mode 1--->'
                print 'cx: %f'%(cx)
                print 'cy: %f'%(cy)
                print 'appear: %s'%(appear)
                print 'area: %f'%(area)
                print 'hit: %f'%(hit)
                print '-----------------------'
                ################################
                if appear :
                    if area <= 0.02:
                        auv.move('forward', cons.AUV_L_SPEED)
                    elif area >= 0.03 or hit >= 0.03:
                        auv.move('backward', cons.AUV_L_SPEED)
                    if self.checkCenter() :
                        print 'center'
                        if area <= 0.02:
                            auv.move('forward', cons.AUV_L_SPEED)
                        elif area >= 0.03 or hit >= 0.03:
                            auv.move('backward', cons.AUV_L_SPEED)
                    else:
                        auv.stop()
                    if 0.02 <= area <= 0.03 and self.checkCenter():
                        print 'colliding with plate'
                        count += 1
                        reset = 0
                    else:
                        reset += 1
                    side_x = -cx
                    side_y = cy
                    print 'sidex: %f'%(side_x)
                    print 'sidey: %f'%(side_y)
                else:
                    print 'move back'
                    auv.multiMove([-cons.AUV_L_SPEED, cons.AUV_L_SPEED*side_x, cons.AUV_L_SPEED*side_y, 0, 0, 0])
                '''
                elif not appear :
                    reset = 0
                    count += 1
                    print 'NOT FOUND PLATE: %d'%(reset)
                    if sidex > 0 :
                        auv.move('right', cons.AUV_L_SPEED*abs(sidex))
                    elif sidex < 0 :
                        auv.move('left', cons.AUV_L_SPEED*abs(sidex))
                    if sidey > 0 :
                        auv.move('forward', cons.AUV_L_SPEED*abs(sidey))
                    elif sidey < 0 :
                        auv.move('backward', cons.AUV_L_SPEED*abs(sidey))
                '''

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

            if mode == 2 :
                print '---mode 2---'
                auv.driveX(1.1)

if __name__ == '__main__' :
    rospy.init_node('buy_gold_chip_node')
    buy_gold_chip = BuyGoldChip()
    buy_gold_chip.run()

