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

def print_result(str):
    print('*'*10 , str ,'*'*10)

class Timer(object):
    def __init__(self):
        self.beginTime = 0
        self.timeout = 0
        self.noOfIncreasing = 3

    def setTimeout(self, sec):
        print_result('Set timeout :'+ str(sec)+ 'second')
        self.beginTime = 0
        self.timeout = sec
        self.beginTime = time.time()

    def isTimeout(self):
        t = time.time() - self.beginTime
        if t >= self.timeout:
            print_result('Timeout')
            return True
        else:
            print_result('Remaining timeout :'+ '%.2f' % (self.timeout - t) + 'second')
            return False

    def increaseTimeout(self, sec):
        if self.noOfIncreasing > 0:
            self.timeout += sec
            self.noOfIncreasing -= 1

    def setLimitIncreasing(self, time):
        self.noOfIncreasing = time

class EstimatePosition():

    def __init__(self,x,y,z,yaw):
        '''
            - x,y,z, and yaw is relativeDistance. if the value is None that means equal to current state
            - Task A to Task B
            - Relative distance between task A and B
            - Previous state is postion task A
            - Next state is position task B
        
        '''
        self.aicontrol = AIControl()
        self.relativeDistance = {'x':x, 'y': y, 'z': z, 'yaw': yaw}
        self.previousState = {'x':0,'y': 0, 'z': z, 'yaw': 0}
        
    def setPreviousState(self):
        auv = self.aicontrol
        self.previousState['x'] = auv.auv_state[0]
        self.previousState['y'] = auv.auv_state[1]
        self.previousState['z'] = auv.auv_state[2]
        self.previousState['yaw'] = auv.auv_state[5]
    
    def moveToEstimatePos(self):
        pass

class ShootCraps(object):
    def __init__(self):
        print_result("SHOOT CRAPS")

        print_result('Wait for service...')
        rospy.wait_for_service('vision_shoot_craps')
        print_result('Service Already')

        self.detectDice = rospy.ServiceProxy(
            'vision_shoot_craps', vision_srv_shoot_craps)

    def request_service(self):
        while not rospy.is_shutdown():
            self.request_dice()

    def request_dice(self):  # , point):
        data = self.detectDice(String('shoot_craps'))  # , point)
        data = data.data
        print_result(data)
        self.dice = {'5': [False], '6': [False], '2': [False]}
        for i in range(len(data.appear)):
            appear = data.appear[i]
            point = data.point[i]
            cx = data.cx[i]
            cy = data.cy[i]
            area = data.area[i]

            self.dice[str(point)] = [appear, cx, cy, area]
        print_result(self.dice)

    # def request_hit(self):
    #     self.hit = False
    #     data = self.detectDice(String('shoot_craps_hit'))
    #     data = data.data
    #     self.hit = data.appear[0]
    #     print_result('Request hit:', self.hit)         

    def isHit(self, point):
        if not self.dice[str(point)][0]:
            return False 

        if  self.dice[str(point)][3] > self.hitArea:
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
    def inRange(self, a,b,c):
        if a <= b <= c:
            return True
        else:
            return False

    '''
        findCheckPoint
            : Passed
    '''
    def findCheckPoint(self):
        self.timer.setTimeout(60)
        auv = self.aicontrol
        nFrame = {'5': 0, '6': 0}
        expectedFrame = 2
        lowSpeed = 0.1
        nextState = False

        while not self.timer.isTimeout():
            auv.move('forward', lowSpeed)
            self.request_dice()
            nFrame['5'] += int(self.dice['5'][0])
            nFrame['6'] += int(self.dice['6'][0])
            if nFrame['5'] > nFrame['6']:
                auv.stop()
                self.move(self.relativeDice['5->6'], lowSpeed)
                self.timer.increaseTimeout(2)
            elif nFrame['6'] > nFrame['5']:
                auv.stop()
                self.move(self.relativeDice['5->6'], lowSpeed)
                self.timer.increaseTimeout(2)
            elif nFrame['6'] >= expectedFrame and nFrame['5'] >= expectedFrame:
                auv.stop()
                nextState = True
                self.checkPoint['x'] = auv.auv_state[0]
                self.checkPoint['y'] = auv.auv_state[1]
                self.checkPoint['yaw'] = auv.auv_state[5]
                break
        print(self.checkPoint)
        return nextState

    def findCheckPointFail(self, doit):
        if not doit:
            return
        # Estimate relative postion  and yaw from Maker

    def hit_dice(self):
        pass       
    '''
        Find dice 5 or 6
    '''

    def move(self,direction,speed):
        auv = self.aicontrol
        for i in range(2):
            auv.move(direction, speed)
    
    def isCenter(self, cx,cy):
        res = []
        if -0.1 <= cx <= 0.1:
            res.append(True)
        else:
            res.append(False)

        if -0.2 <= cy <= 0.2:
            res.append(True)
        else:
            res.append(False)
        return res

    def hitDice(self, point):
        self.timer.setTimeout(240)
        auv = self.aicontrol
        lowSpeed = 0.2

        while not self.timer.isTimeout():
            if not self.isHit(point):
                self.request_dice()
                if self.dice[str(point)][0]:
                    appear, cx, cy, area = self.dice[str(point)] 
                    print_result(str(cx)+' '+str(cy)+' '+str(area))
                    checkCenter = self.isCenter(cx,cy)
                    if not checkCenter[0]: 
                        auv.stop()            		
                        if cx > 0:
                            print_result('Right')
                            # Constant from Bro!!s
                            self.move('right', (cx + 1)*1.5)
                        elif cx < 0:
                            print_result('Left')
                            self.move('left', (abs(cx) + 1)*1.5)

                    if not checkCenter[1]:
                        auv.stop()            		
                        if cy < 0: 
                            print_result('Up too me')
                            self.move('up', abs(cy) / 20.)
                        elif cy > 0:
                            print_result('Down depend on control')
                            for i in range(int(cy*7.5)):
                                self.move('down', 1)
                        auv.depthRelative(0)
                    if checkCenter[0] == True and checkCenter[1] == True and self.isHit(point):
                        print_result('center and before hit')
                        self.hit_dice()
                        auv.stop()
                        return True                   
                else:
                    print_result('Not see everything BLIIND!!!!')
                    self.move('forward', 0.1)
            else:
                print_result('Before hit!!!!')
                self.hit_dice()
                auv.stop()
                return True
        return False

    '''
        Create new method because in the future, will be added state
    '''
    def hitDiceFail(self,doit):
        if not doit:
            self.back2CheckPoint()
    
    
    '''
        back to check point 
    '''

    def back2CheckPoint(self):
        auv = self.aicontrol
        auv.driveX(-2)
        auv.fixXY(self.checkPoint['x'], self.checkPoint['y'])     
        auv.turnAbs(math.degrees(self.checkPoint['yaw']))
    
    '''
        1. Find checkpoint
        2. hit 5
        3. if hit 5 fail -> hit 6 -> hit 5 again -> if fail "Next Mission" 
        4. else hit 6 
        5. hit 6 fail -> hit 5 -> hit 6 again -> if fail "Next Mission"
    '''
    def run(self):
        print_result('REQUEST SHOOT CRAPS SERVICE')
        while not rospy.is_shutdown():
            self.request_dice()


if __name__ == '__main__':
    rospy.init_node('shoot_craps', anonymous=False)
    print_result("INIT NODE")
    SC = ShootCraps()
    # SC.run()
    SC.request_service()
    #SC.hitDice(5)
    # SC.findCheckPoint()
    # while not rospy.is_shutdown():
    #     SC.request_hit()

