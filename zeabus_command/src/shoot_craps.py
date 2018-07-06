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
        print 'Set timeout :', sec, 'second'
        self.beginTime = 0
        self.timeout = sec
        self.beginTime = time.time()

    def isTimeout(self):
        t = time.time() - self.beginTime
        if t >= self.timeout:
            print 'Timeout'
            return True
        else:
            print 'Remaining timeout :', '%.2f' % (self.timeout - t), 'second'
            return False


class ShootCraps(object):
    def __init__(self):
        print("SHOOT CRAPS")
        self.aicontrol = AIControl()

        print('Wait for service...')
        rospy.wait_for_service('vision_shoot_craps')
        print('Service Already')
        self.data = None
        self.checkPoint = [0, 0]
        self.detect_dice = rospy.ServiceProxy(
            'vision_shoot_craps', vision_srv_shoot_craps)
        self.timer = Timer()
        self.dice = {'5': [False], '6': [False], '2': [False]}
        self.relativeDice = {'5->6':'left','6->5':'right'}

    def request(self):  # , point):
        self.data = self.detect_dice(String('shoot_craps'))  # , point)
        self.data = self.data.data
        self.dice = {'5': [False], '6': [False], '2': [False]}

        for i in range(len(self.data.appear)):
            appear = self.data.appear
            point = self.data.point
            cx = self.data.cx
            cy = self.data.cy
            area = self.data.area

            self.dice[str(point)] = [appear, cx, cy, area]

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
        auv = self.aicontrol
        # nFrame = {'2':0,'5':0,'6':0}
        nFrame = {'5':0,'6':0}
        lowSpeed = 0.2
        # highSpeed = 0.7

        while not self.timer.isTimeout():
            auv.move('forward',lowSpeed)
            self.request()
            # nFrame['2'] += int(self.dice['2'][0])
            nFrame['5'] += int(self.dice['5'][0])
            nFrame['6'] += int(self.dice['6'][0])
            if nFrame['5'] > nFrame['6']:
                auv.move(relativeDice['5->6'],lowSpeed)
            elif nFrame['6'] > nFrame['5']:
                auv.move(relativeDice['5->6'],lowSpeed)

        if nFrame['5'] >= 3 and  nFrame['6'] >= 3:
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
