#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
#from _sonar_msg import *
#from _sonar_srv import *
from zeabus_imaging_sonar.srv import sonar_markersrv
sonar_image = None

def server(start):
    global sonar_image
    # try:      
    response = sonar_image(start)
    print response
    return response.theta, response.r, response.status
    # except  rospy.ServiceException, e:
    #     print "Service call failed: %s"%e


if __name__ == "__main__":
    print "Waiting"
    start = True
    count = 0
#    global sonar_image
    rospy.wait_for_service('/sonar_image')
    sonar_image = rospy.ServiceProxy('/sonar_image', sonar_markersrv)
    print 'service start'
    while not rospy.is_shutdown():
        count += 1
        time.sleep(1)
        response = sonar_image(1.4,5.3)
        """for i, (ri, thetai) in enumerate(zip(response.r, response.theta)):
            thetai = thetai-90
            print "(%d, %d)" %(ri, thetai)""" 
	#print (response.data.r)    
        print response







