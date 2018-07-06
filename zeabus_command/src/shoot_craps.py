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
        self.noOfIncreasing = 3

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

    def increaseTimeout(self, sec):
        if self.noOfIncreasing > 0:
            self.timeout += sec
            self.noOfIncreasing -= 1

    def setLimitIncreasing(self, time):
        self.noOfIncreasing = time


class ShootCraps(object):
    def __init__(self):
        print("SHOOT CRAPS")
        self.aicontrol = AIControl()

        print('Wait for service...')
        rospy.wait_for_service('vision_shoot_craps')
        print('Service Already')
        self.data = None
        self.checkPoint = {'x': 0, 'y': 0, 'yaw': 0}
        self.detect_dice = rospy.ServiceProxy(
            'vision_shoot_craps', vision_srv_shoot_craps)
        self.timer = Timer()
        self.dice = {'5': [False], '6': [False], '2': [False]}
        self.relativeDice = {'5->6': 'left', '6->5': 'right'}
        # wait for testing
        self.hitArea = 1

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

    def isHit(self, point):
        if not self.dice[str(point)][0]:
            return False 

        if  self.dice[str(point)][0][3] > self.hitArea:
            return True
        return False

    '''
        Find checkpoint by detect dice 5 and 6
        Condition
            - Low speed
            - Cener between 5 and 6
        Break
            - Distance more than X

    '''

    def findCheckPoint(self):
        self.timer.setTimeout(10)
        auv = self.aicontrol
        # nFrame = {'2':0,'5':0,'6':0}
        nFrame = {'5': 0, '6': 0}
        expectedFrame = 3
        lowSpeed = 0.2
        nextState = False
        # highSpeed = 0.7

        while not self.timer.isTimeout():
            auv.move('forward', lowSpeed)
            self.request()
            # nFrame['2'] += int(self.dice['2'][0])
            nFrame['5'] += int(self.dice['5'][0])
            nFrame['6'] += int(self.dice['6'][0])
            if nFrame['5'] > nFrame['6']:
                auv.move(self.relativeDice['5->6'], lowSpeed)
                self.timer.increaseTimeout(2)
            elif nFrame['6'] > nFrame['5']:
                auv.move(self.relativeDice['5->6'], lowSpeed)
                self.timer.increaseTimeout(2)
            elif nFrame['6'] >= expectedFrame and nFrame['5'] >= expectedFrame:
                nextState = True
                self.checkPoint['x'] = auv.auv_state[0]
                self.checkPoint['y'] = auv.auv_state[1]
                self.checkPoint['yaw'] = auv.auv_state[5]
                break

        return nextState

    def findCheckPointFail(self, doit):
        if not doit:
            return
        # Estimate relative postion  and yaw from Maker

    '''
        Find dice 5 or 6
    '''

    def hitDice(self, point):
        self.timer.setTimeout(10)
        auv = self.aicontrol

        while not self.timer.isTimeout():
            auv.move('forward', lowSpeed)
            if self.isHit(5):
                auv.driveX(1)
                break
    '''
        back to check point 
    '''

    def back2CheckPoint(self, x):
        auv = self.aicontrol
        auv.driveX(-x)
        auv.fixXY(self.checkPoint['x'], self.checkPoint['y'])     

    '''
        Predict dice position by previous position (Forwarding) 
        if cannot detect dice more than X frame
    '''

    def predictDicePosition(self):
        pass

    '''
        1. Find checkpoint
        2. hit 5
        3. if hit 5 fail -> hit 6 -> hit 5 again -> if fail "Next Mission" 
        4. else hit 6 
        5. hit 6 fail -> hit 5 -> hit 6 again -> if fail "Next Mission"
    '''
    def run(self):
        print('SHOOT CRAPS RUNNING')
        nextState = self.findCheckPoint()
        self.findCheckPointFail(nextState)

        nextState = self.hitDice(5)


if __name__ == '__main__':
    rospy.init_node('shoot_craps', anonymous=False)
    print("INIT NODE")
    SC = ShootCraps()
    SC.run()
