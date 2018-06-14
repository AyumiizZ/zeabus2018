#!/usr/bin/python2.7

import rospy
import constants as cons
from aicontrol import AIControl
from zeabus_vision.srv import vision_srv_qualifying_marker
from zeabus_vision.msg import vision_qualifying_marker
from std_msgs.msg import String, Float64

class Marker(object):
    def __init__(self):
        print '<===INIT MARKER===>'

        print '---wait for vision service---'
        rospy.wait_for_service('vision_qualifying_marker')
        self.marker_req = rospy.ServiceProxy('vision_qualifying_marker', vision_srv_qualifying_marker)
        self.data = vision_qualifying_marker()
        self.pub = rospy.Publisher('/Marker/data', Float64, queue_size=10)

    def detectMarker(self):
        self.data = self.marker_req(String('marker'))
        self.data = self.data.data

    def run(self):
        while not rospy.is_shutdown():
            self.detectMarker()
            if self.data.appear:
                self.pub.publish(Float64(self.data.area))
            print self.data

if __name__=='__main__':
    rospy.init_node('marker_node')
    marker = Marker()
    marker.run()
