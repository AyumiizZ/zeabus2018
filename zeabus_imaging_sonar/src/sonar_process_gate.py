#!/usr/bin/env python
from __future__ import division
import sys
import cv2
import time
import numpy as np
import roslib
import rospy
import math
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from zeabus_imaging_sonar.srv import sonar_gatesrv
from zeabus_imaging_sonar.msg import sonar_gatemsg
from cv_bridge import CvBridge, CvBridgeError

#from sonar.msg import sonar_msg
#from _sonar_msg import *
#from zeabus_imaging_sonar.sr53v import sonar_srv
#from _sonar_srv import *

# Instantiate CvBridge
bridge = CvBridge()

height, width = 660, 768
#height, width = 330, 384
Img_frame, preImg_gray = None, None
count = 0
index = 0
p0, p1 = [], []
new_pos, old_pos = [], []
good_new, good_old = [], []

r_out, theta_out = [], []

r_thres = 14

r_met = 0
status = False
theta_met = 0

mask = None
X_now = 0
Y_now = 0

def findobject(Img_gray):
	global imgBlur, kernel, npower, ratio, imgcfar, reEdge, opening, X_now, Y_now, Xnow_right, Ynow_right

	imgBlur = cv2.medianBlur(Img_gray,5)
	imgBlur = imgBlur.astype('float32')
	kernel = np.ones((11,11),'float32')/40.0
	kernel[1:10,1:10] = 0
	npower = cv2.filter2D(imgBlur,cv2.CV_8UC1,kernel)
	npower = npower.astype('float32')+1.0
	ratio = imgBlur/npower
	imgcfar = (ratio>1.45)*255
	imgcfar = imgcfar.astype('uint8')

	#remove noise on edge 
	deEdge = np.zeros((height,width), np.uint8)
	deEdge = cv2.rectangle(deEdge,(0,0),(height,width),(255,255,255),400)
	reEdge = imgcfar-deEdge
	
	#opening
	kernelOpen = np.ones((17,17),np.uint8)
	opening = cv2.morphologyEx(reEdge, 2, kernelOpen)

	#Skeletion--------------------------------------------------
	size = np.size(opening)
	skel = np.zeros(opening.shape,np.uint8)
	ret,img = cv2.threshold(opening,127,255,0)
	element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
	done = False

	while( not done):
    		eroded = cv2.erode(opening,element)
    		temp = cv2.dilate(eroded,element)
    		temp = cv2.subtract(opening,temp)
    		skel = cv2.bitwise_or(skel,temp)
    		opening = eroded.copy()
    		zeros = size - cv2.countNonZero(opening)
    		if zeros==size:
        		done = True


# Copy edges to the images that will display the results in BGR
	cdstP = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
	linesP = cv2.HoughLinesP(skel, 1, np.pi / 180, 50, None, 50, 10)
    
	if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

	#cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
	
	#cv2.waitKey(0)
	
	#pubIm(opening)
	pubIm(cdstP)
#return cdstP

#-----------------------------------------------------------------------------------------------------

	X1 = l[0]
	Y1 = l[1]
	X2 = l[2]
	Y2 = l[3]

#find location on the left
	theta = (2/width)*60*(X1-(width/2))
	r = (height-Y1)
		
	xnew = r*math.sin(math.radians(theta))+(1270/2)
	ynew = 680-r*math.cos(math.radians(theta))

#find answer r,theta
	X = xnew
	Y = ynew
	delX = X-(1270/2)
	delY = 680-Y

	r_new = ((math.sqrt(math.pow(delX,2)+math.pow(delY,2)))*18)/620
	theta_new = math.degrees(math.atan((math.fabs(delX)/delY)))
	
	X_now = r_new*math.sin(math.radians(theta_new))
	Y_now = r_new*math.cos(math.radians(theta_new))
	print 'Xnow = ''%.2f'%X_now,'m , Ynow = ''%.2f'%Y_now, 'm'
			
	
	if delX >= 0 :
		print 'r = ''%.2f'%r_new ,'m , theta = ''%.2f'%theta_new,'degrees'
		print '########################################'
	
	
	else  :
		print 'r = ''%.2f'%r_new ,'m , theta = ''%.2f'%(-1*theta_new),'degrees'
		print '########################################'
	
	
#find location on the right 			
	theta_right = (2/width)*60*(X2-(width/2))
	r_right = (height-Y2)
			
	xnew_right = r_right*math.sin(math.radians(theta_right))+(1270/2)
	ynew_right = 680-r_right*math.cos(math.radians(theta_right))
	
	X_right = xnew_right
	Y_right = ynew_right
	delX_right = X_right-(1270/2)
	delY_right = 680-Y_right
	r_new_right = ((math.sqrt(math.pow(delX_right,2)+math.pow(delY_right,2)))*18)/620
	theta_new_right = math.degrees(math.atan((math.fabs(delX_right)/delY_right)))
	
	Xnow_right = r_new_right*math.sin(math.radians(theta_new_right))
	Ynow_right = r_new_right*math.cos(math.radians(theta_new_right))
	print 'Xnow_right = ''%.2f'%Xnow_right,'m , Ynow_right = ''%.2f'%Ynow_right, 'm'
			
	
	if delX >= 0 :
		print 'r_right = ''%.2f'%r_new_right ,'m , theta_right = ''%.2f'%theta_new_right,'degrees'
		print '########################################'
	
	
	else  :
		print 'r_right = ''%.2f'%r_new_right ,'m , theta_right = ''%.2f'%(-1*theta_new_right),'degrees'
		print '########################################'
	
	cv2.waitKey(0)		
	
	
def pubIm(im):
	img_show = np.array(im, np.uint8)
	msg1 = CompressedImage()
	msg1.format = "jpeg"
	msg1.header.stamp = rospy.Time.now()
	msg1.data = np.array(cv2.imencode('.jpg', img_show)[1]).tostring()
	pub.publish(msg1)

def Process():
	global Img_frame, preImg_gray, Img_show, count, p0, p1, good_new, good_old, r_met, theta_met, status, mask, new_pos, old_pos
	global r_out, theta_out
	new_pos, old_pos = [], []
	last_frame = False
	res = sonar_gatemsg()  

	while(Img_frame is None):
		print "Img_frame: None"
		rospy.sleep(0.01)
		continue
	
	
	Img_gray = cv2.cvtColor(Img_frame, cv2.COLOR_BGR2GRAY)
	p0 = findobject(Img_gray)
	

	res.X_left = X_now
	res.Y_left = Y_now
	res.status = status
	res.X_right = Xnow_right
	res.Y_right = Ynow_right
	
	if p0 == (-1,-1):res.status = False
	else : res.status = True

	print X_now
	print Y_now
	print status
	print Xnow_right
	print Ynow_right
	
	cv2.waitKey(1)
	return res
	
	
def image_callback(ros_data):
	global Img_frame, width, height, index
	index += 1
	#print "index = %s" %index
	try:
		Img_frame = cv2.resize(bridge.imgmsg_to_cv2(ros_data, "bgr8"),(width, height))
	except CvBridgeError, e:
		print (e)

def tracking_callback(msg):
	print msg
	print "tracking callback"
	return Process()

if __name__ == '__main__':
	rospy.init_node('SonarTracking', anonymous=True)
	## Publish for show an image
	pub = rospy.Publisher('/image/Tracking', CompressedImage, queue_size=1)
	#connect or open (connect_image_sonar,imaging_sonar)
	subTopic = "/imaging_sonar"
	sub = rospy.Subscriber(subTopic, Image, image_callback,  queue_size = 1)
	rospy.Service('/sonar_image', sonar_gatesrv, tracking_callback)
	print("finished setup")
	rospy.spin()
