#!/usr/bin/python2.7

import rospy
import constants as cons
from aicontrol import AIControl
from zeabus_vision.srv import vision_srv_qualifying_marker
from zeabus_vision.msg import vision_qualifying_marker
from zeabus_imaging_sonar.srv import sonar_srv
from zeabus_imaging_sonar.msg import sonar_msg
from std_msgs.msg import String, Float64

class Marker(object):
    def __init__(self):
        print '<===INIT MARKER===>'
        self.aicontrol = AIControl()

        print '---wait for sonar service---'
        rospy.wait_for_service('sonar_image')
        self.marker_req = rospy.ServiceProxy('/sonar_image', sonar_srv)
        self.data = sonar_msg()

        print '---wait for vision service---'
        rospy.wait_for_service('vision_qualifying_marker')
        self.marker_req = rospy.ServiceProxy('vision_qualifying_marker', vision_srv_qualifying_marker)
        self.data = vision_qualifying_marker()


    def detectMarker(self):
        self.data = self.marker_req(String('marker'))
        self.data = self.data.data

    def run(self):
        while not rospy.is_shutdown():
            self.detectMarker()
            print self.data

if __name__=='__main__':
    rospy.init_node('marker_node')
    marker = Marker()
    marker.run()

