#!/usr/bin/python2.7

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_example.srv import vision_srv_path
from zeabus_example.msg import vision_path
form aicontrol import AIControl
import constants as cons

class Path(object) :

    def __init__(self) :
        self.data = vision_path
        print '<===INIT PATH===>'
        self.aicontrol = AIcontrol()
        rospy.wait_for_service('vision_path')
        self.detect_path = rospy.ServiceProxy('vision_path', vision_srv_path)



    def detectPath(self) :
        self.data = self.detect_path(String('path'), String('path'))
        self.data = self.data.data

    
    def run(self) :
        auv = self.aicontrol
        print '<===DOING PATH===>'

