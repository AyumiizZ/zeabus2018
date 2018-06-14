#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from zeabus_vision.msg import vision_dice
from zeabus_vision.srv import vision_srv_dice

class dice(object):
    def __init__(self):
        #self.aicontrol = AIControl()
        self.data = vision_dice
        #rospy.wait_for_service('vision_dice')
        self.detect_dice = rospy.ServiceProxy('vision_dice', vision_srv_dice)
#    def detectDice(self) :
 #       self.data = self.detect_dice(String('dice'))
  #      self.data = self.data.data

if __name__=='__main__'    rospy.init_node('shoot_craps', anonymous=False)
    dice = dice()
  #  dice.detectDice()
    rospy.spin()



