#!/usr/bin/python2.7

import rospy
from std_msgs.msg import String, Float64, Bool
from zeabus_vision.srv import vision_srv_path
from zeabus_vision.msg import vision_path
from aicontrol import AIControl
import constants as cons

class Path(object) :

    def __init__(self) :
        self.data = vision_path
        print '<===INIT PATH===>'
        self.aicontrol = AIControl()
        rospy.wait_for_service('vision_path')
        self.detect_path = rospy.ServiceProxy('vision_path', vision_srv_path)

    def detectPath(self) :
        self.data = self.detect_path(String('path'))
        self.data = self.data.data

    def run(self) :
        while not rospy.is_shutdown():
            self.detectPath()
            print
            print '-------------------'
            print self.data.cx
            print self.data.cy
            print self.data.appear
            print self.data.degrees
            print '-------------------'

if __name__=='__main__':
    rospy.init_node('path_node')
    path = Path()
    path.run()
