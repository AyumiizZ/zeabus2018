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
        auv.depthAbs(cons.PATH_DEPTH)

        mode = 0
        count = 0
        center = 0 

        while not rospy.is_shutdown() and not mode == -1:
            #get data from vision service and store in temp variables
            self.detectPath()
            area = self.data.area
            appear = self.data.appear
            cx = self.data.cx
            angle = self.data.angle

            #find path
            if mode == 0 :
                print '<---MODE 0--->'

                #check if path appear
                if appear :
                    count += 1
                     reset = 0
                    print 'FOUND PATH: %d'%(count)
                elif not appear:
                    reset += 1
                    print 'NOT FOUND PATH: %d'%(reset)

                # check counter
                if count >= 5:
                    count = 0
                    reset = 0
                    print '<<<Change to mode 1>>>'
                    mode = 1
                elif reset >= 5:
                    reset = 0
                    count = 0

                auv.move('forward', cons.AUV_M_SPEED)

            if mode == 1 :
                print '<---MODE 1--->'
                print 'cx: %f'%(cx)
                print 'area: %f'%(area)
                if cx < 0:
                    auv.move('left', cons.AUV_M_SPEED*abs(cx))
                elif cx > 0:
                    auv.move('right', cons.AUV_M_SPEED*abs(cx))
                #check if auv is center of path or not
                if -cons.VISION_PATH_ERROR <= cx <= cons.VISION_PATH_ERROR:
                    print '<<<CENTER>>>'
                    center += 1
                    auv.stop()
                elif -cons.VISION_PATH_ERROR > cx > cons.VISION_PATH_ERROR:
                    reset += 1
                #check center's counter
                if center >= 3:
                    print '<<<Change to mode 2>>>'  
                    mode = 2
                elif reset >= 10:
                    center = 0
                    reset = 0

            #go on path
            if mode == 2 :
                print '<---MODE 2--->'
                auv.move('forward', cons.AUV_M_SPEED)
                auv.turnAbs(angle,cons.VISION_PATH_ERROR)
