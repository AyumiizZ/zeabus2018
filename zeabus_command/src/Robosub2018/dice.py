#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from zeabus_example.msg import vision_dice
from zeabus_example.srv import vision_srv_dice

class Dice(object):
    def __init__(self):
        self.aicontrol = AIControl()


if __name__=='__main__':
    rospy.init_node('dice_node')
