#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from std_msgs.msg import String
from aicontrol import AIControl
from zeabus_vision.msg import vision_shoot_craps
from zeabus_vision.srv import vision_srv_shoot_craps
import time

'''
    Use dice in singular and plural
'''

class Timer(object):
    def __init__(self):
        self.beginTime = 0
        self.timeout = 0

    def setTimeout(self, sec):
        self.beginTime = 0
        self.timeout = sec
        self.beginTime = time.time()

    def isTimeout(self):
        if time.time() - self.beginTime >= self.timeout:
            return True
        else:
            return False

class ShootCraps(object):
    def __init__(self):
        print("SHOOT CRAPS")
        # self.aicontrol = AIControl()
        print('Wait for service...')
        rospy.wait_for_service('vision_shoot_craps')
        print('Service Already')
        self.data = None
        self.checkPoint = [0,0]
        self.detect_dice = rospy.ServiceProxy(
            'vision_shoot_craps', vision_srv_shoot_craps)
        self.timer = Timer()

    def request(self):#, point):
        self.data = self.detect_dice(String('shoot_craps'))#, point)
        self.data = self.data.data

    def isHit(self):
        pass

    '''
        Find all dice for set the checkpoint
        Condition
            - Low speed
            - Cener between 5 and 6
        Break
            - Distance more than X

    '''

    def detectAllDice(self):
        self.timer.setTimeout(10)

        while not self.timer.isTimeout():
            self.request()
        print('Timeout')

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
        print('SHOOT CRAPS RUNNING')
        self.detectAllDice()
        pass


if __name__ == '__main__':
    rospy.init_node('shoot_craps', anonymous=False)
    print("INIT NODE")
    SC = ShootCraps()
    SC.run()
