#!/usr/bin/python2

import rospy
import constants as cons
from aicontrol import AIControl
from zeabus_vision.srv import vision_srv_dice
from zeabus_vision.msg import vision_dice
from std_msgs.msg import String

class Dice(object):
    def __init__(self):
        print '<==INIT DICE==>'
        self.aicontrol = AIControl()

        print '--wait for vision service--'
        rospy.wait_for_service('vision_dice')
        self.dice_req = rospy.ServiceProxy('vision_dice', vision_srv_dice)
        self.dice_data = vision_dice

    def detect(self, num):
        self.dice_data = self.dice_req(String('dice'), String(str(num)))
        self.dice_data = self.dice_data.data

    def run(self):
        auv = self.aicontrol

        print '<==Doing Dice==>'

        mode = 0
        count = 0
        reset = 0

        while not rospy.is_shutdown() and not mode == -1:
            self.detect()
            cx = self.dice_data.cx
            cy = self.dice_data.cy

            if mode == 0:
                print '---mode 0---'

                if appear:
                    count += 1
                    reset = 0
                    print 'FOUND DICE: %d'%(count)
                else:
                    reset += 1
                    print 'NOT FOUND DICE: %d'%(reset)

                if count >= 5:
                    count = 0
                   reset = 0
                    print '<<<Change to mode 1>>>'
                elif reset >= 5:
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)

if __name__ == '__main__':
    rospy.init_node('dice_node')
    dice = Dice()
    dice.run()
