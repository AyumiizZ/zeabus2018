#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl

class Qualify(object):
    def __init__(self):
        self.aicontrol = AIControl()

    def run(self):
        
