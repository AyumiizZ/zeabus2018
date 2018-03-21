#!/usr/bin/python2.7

import rospy
from aicontrol import AIControl
from zeabus_example.srv import vision_srv_gate
from zeabus_example.msg import vision_gate

class Gate(object):
    def __init__(self):
        self.aicontrol = AIControl()
        

if __name__=='__main__':
    rospy.init_node('gate_node')
