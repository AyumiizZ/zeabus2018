#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl

class Qualify(object):
    def __init__(self):
        self.aicontrol = AIControl()

    def run(self):
        auv = self.aicontrol
        auv.depthAbs(cons.QUALIFY_DEPTH)
        auv.driveX(5)
        auv.driveY(2)
        auv.driveX(9)
        auv.turnRelative(-90)
        auv.driveX(4)
        auv.turnRelative(-90)
        auv.driveX(5)
        auv.driveY(-2)
        auv.driveX(10)

if __name__=='__main__':
    rospy.init_node('qualify_node')
    qualify = Qualify()
    qualify.run()
