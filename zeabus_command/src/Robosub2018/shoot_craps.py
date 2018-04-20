#!/usr/bin/python2

import rospy
from aicontrol import AIControl
from zeabus_example.msg import dice_msg
from zeabus_example.srv import dice_srv

class Shoot_craps(object):
    def __init__:
        self.aicontrol = AIControl()

if __name__=='__main__':
    rospy.init_node('shoot_craps')
