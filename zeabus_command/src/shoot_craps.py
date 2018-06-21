#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from zeabus_vision.msg import vision_dice
from zeabus_vision.srv import vision_srv_dice

class dice(object):
    def __init__(self):
        self.aicontrol = AIControl()
        self.data = vision_dice
        rospy.wait_for_service('vision_dice')
        self.detect_dice = rospy.ServiceProxy('vision_dice', vision_srv_dice)

    def detectDie(self, point) :
        self.data = self.detect_dice(String('dice'), point)
        self.data = self.data.data

    def nextDie(self, cur):
        if cur == 1:
            return 6
        elif cur == 2:
            return 5
        elif cur == 5:
            return 6
        elif cur == 6:
            return 5

    def run(self):
        mode = 0
        count = 0
        reset = 0

        target_die = 0
        found = False

        checkpoint = [0, 0]

        count_dice = [0, 0, 0, 0]
        reset_count = [0, 0, 0, 0]

        dice = {0:1, 1:2, 2:5, 3:6}

        while not rospy.is_shutdown() and mode != -1:
            if mode == 0:
                print 'Mode 0'
                for index in dice:
                    auv.move('forward', cons.AUV_M_SPEED)
                    print 'Look for %d point'%dice[index]
                    self.detectDie(dice[index])
                    found = self.data.appear
                    if found:
                        count_dice[index] += 1
                        reset_count[index] = 0
                    else:
                        reset_count[index] += 1

                    if reset_count[index] >= 5:
                        count_dice[index] = 0
                        reset_count[index] = 0

                    if count_dice[index] >= 5:
                        target_die = dice[index]
                        print 'Found %d point'%(target_die)
                        print 'Set target to %d'%(target_die)
                        checkpoint = [auv.auv_state[0], auv.auv_state[1]]
                        print '---checkpoint---'
                        print checkpoint
                        mode = 1
                        break

            if mode == 1:
                print 'Mode 1'
                self.detectDie(target_die)
                found = self.data.appear
                cx = self.data.cx
                cy = self.data.cy
                print '<<<Data from Vision>>>'
                print 'APPEAR: %s'%(found)
                print 'CX: %f'%(cx)
                print 'CY: %f'%(cy)
                if found:
                    auv.multiMove([1, cx, cy, 0, 0, 0])
                    reset += 1
                else:
                    count += 1
                    reset = 0

                if count >= 5:
                    target_die = nextDie(target_die)
                    print 'next target is %d'%(target_die)
                    auv.backTrack(checkpoint[0], checkpoint[1])
                    mode = 2
                if reset >= 5:
                    count = 0
                    reset = 0

            if mode == 2:
                self.detectDie(target_die)
                found = self.data.appear
                cx = self.data.cx
                cy = self.data.cy
                print '<<<Data from Vision>>>'
                print 'APPEAR: %s'%(found)
                print 'CX: %f'%(cx)
                print 'CY: %f'%(cy)
                if found:
                    auv.multiMove([1, cx, cy, 0, 0, 0])
                    reset += 1
                else:
                    count += 1
                    reset = 0

                if count >= 5:
                    mode = 3
                if reset >= 5:
                    count = 0
                    reset = 0

            if mode == 3:
                auv.driveX(-4)
                auv.backTrack(checkpoint[0], checkpoint[1])
                auv.depthAbs(-1)
                auv.driveX(5)

if __name__=='__main__':
    rospy.init_node('shoot_craps')
    dice = dice()
