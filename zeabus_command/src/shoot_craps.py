#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from std_msgs.msg import String
from aicontrol import AIControl
from zeabus_vision.msg import vision_shoot_craps
from zeabus_vision.srv import vision_srv_shoot_craps

'''
    Use dice in singular and plural
'''


class ShootCraps(object):
    def __init__(self):
        self.aicontrol = AIControl()
        rospy.wait_for_service('vision_shoot_craps')
        self.detect_dice = rospy.ServiceProxy(
            'vision_dice', vision_srv_shoot_craps)

    def detectDie(self, point):
        self.data = self.detect_dice(String('shoot_craps'), point)
        self.data = self.data.data

    def isHit(self):
        pass

    '''
        Find all dice for set the checkpoint
    '''

    def detectAllDice(self):
        pass

    '''
        Find dice 5 or 6
    '''

    def detectDice(self):
        pass

    '''
        back to check point 
    '''

    def back2CheckPoint(self):
        pass

    '''
        Predict dice position by previous position (Forwarding) 
        if cannot detect dice more than X frame
    '''

    def predictDicePosition(self):
        pass

    def run(self):
        pass


if __name__ == '__main__':
    rospy.init_node('shoot_craps')
    SC = ShootCraps()
    SC.run()
