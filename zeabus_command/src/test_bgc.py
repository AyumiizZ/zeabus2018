#!/usr/bin/python2.7
import rospy
from zeabus_vision.msg import vision_buy_a_gold_chip
from zeabus_vision.srv import vision_srv_buy_a_gold_chip
from aicontrol_sim import AIControl
from std_msgs.msg import String, Float64, Bool
import constants as cons
class BuyGoldChip(object) :

    def __init__(self) :
        print '<===INIT BUY_GOLD_CHIP===>'
        self.data = vision_buy_a_gold_chip()
        self.plate = vision_buy_a_gold_chip()
        self.ball = vision_buy_a_gold_chip()

        print 'wait for vision service'
        rospy.wait_for_service('vision_buy_a_gold_chip')
        self.detect_buy_a_gold_chip = rospy.ServiceProxy('vision_buy_a_gold_chip', vision_srv_buy_a_gold_chip)

    def detectPlate(self) :
        try:
            self.data = self.detect_buy_a_gold_chip(String('buy_a_gold_chip'), String('front'))
            self.plate = self.data.data
        except: print 'Error'

    def detectBall(self) :
        self.data = self.detect_buy_a_gold_chip(String('buy_a_gold_chip'), String('bottom'))
        self.ball = self.data.data

    def run(self):
        while not rospy.is_shutdown():
            self.detectPlate()
            cx = self.plate.cx
            cy = self.plate.cy
            appear = self.plate.appear
            area = self.plate.area
            hit = self.plate.hit
            print '-----------------------'
            print 'cx: %f'%(cx)
            print 'cy: %f'%(cy)
            print 'appear: %s'%(appear)
            print 'area: %f'%(area)
            print 'hit: %f'%(hit)
            print '-----------------------'

if __name__ == '__main__' :
    rospy.init_node('buy_gold_chip_node')
    buy_gold_chip = BuyGoldChip()
    buy_gold_chip.run()

