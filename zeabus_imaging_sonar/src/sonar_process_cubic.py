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
from zeabus_imaging_sonar.srv import sonar_cubicsrv
from zeabus_imaging_sonar.msg import sonar_cubicmsg
from cv_bridge import CvBridge, CvBridgeError

#from sonar.msg import sonar_msg
#from _sonar_msg import *
#from zeabus_imaging_sonar.sr53v import sonar_srv
#from _sonar_srv import *

# Instantiate CvBridge
bridge = CvBridge()

#sonar.sh
height, width = 660, 768
#height, width = 330, 384
Img_frame, preImg_gray = None, None
count = 0
index = 0
p0, p1 = [], []
new_pos, old_pos = [], []
good_new, good_old = [], []

r_out, theta_out = [], []
#x_out, y_out = [], []

#theta_inrange = 0
#distant_inrange = 0

r_thres = 14

r_met = 0
status = False
theta_met = 0

mask = None



def findobject(Img_gray):
	global imgBlur, kernel, npower, ratio, imgcfar, reEdge, opening, theta_out, r_out,x_out, y_out

	imgBlur = cv2.medianBlur(Img_gray,5)
	imgBlur = imgBlur.astype('float32')
	kernel = np.ones((11,11),'float32')/102
	kernel[4:7,4:7] = 0
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
	kernelOpen = np.ones((3,3),np.uint8)
	opening = cv2.morphologyEx(reEdge, 2, kernelOpen)
	
	x_out = []
	y_out = []
	r_out = []
	theta_out = []
	#find center pixel  
	ret,thresh = cv2.threshold(opening,127,255,0)
	contours = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#when cannot find object
	r_new = theta_new = -1
	#when can find object
	for contour in contours[1] :
		x = contour.mean(0)[0,0]
		y = contour.mean(0)[0,1]
		
		#find location
		theta = (2/width)*60*(x-(width/2))
		r = (height-y)
		
		#xnew = r*math.sin(math.radians(theta))+(941/2)
		#ynew = 491-r*math.cos(math.radians(theta))
		#RawImage X = 1270, Y = 680		
		xnew = r*math.sin(math.radians(theta))+(1268/2)
		ynew = 678-r*math.cos(math.radians(theta))
		
		#find answer r,theta
		X = xnew
		Y = ynew
		#delX = X-(941/2)
		#delY = 491-Y
		delX = X-(1268/2)
		delY = 678-Y
		
		
		#r_new = ((math.sqrt(math.pow(delX,2)+math.pow(delY,2)))*2)/116
		r_new = ((math.sqrt(math.pow(delX,2)+math.pow(delY,2)))*18)/620
		theta_new = math.degrees(math.atan((math.fabs(delX)/delY)))
		Xnow = r_new*math.sin(math.radians(theta_new))
		Ynow = r_new*math.cos(math.radians(theta_new))
		print 'Xnow = ''%.2f'%Xnow,'m , Ynow = ''%.2f'%Ynow, 'm'

		#print (x_out,y_out,r_out,theta_out)
		x_out.append(Xnow)
		y_out.append(Ynow)
		r_out.append(r_new)
		theta_out.append(theta_new)
		if delX >= 0 :
			print 'r = ''%.2f'%r_new ,'m , theta = ''%.2f'%theta_new,'degrees'
			print '########################################'
			#theta_out.append(theta_new)

		else  :
			print 'r = ''%.2f'%r_new ,'m , theta = ''%.2f'%(-1*theta_new),'degrees'
			print '########################################'
			#theta_out.append(-1*theta_new)
			
	pubIm(opening)

	
	

	
	return r_new,theta_new

def pubIm(im):
	img_show = np.array(im, np.uint8)
	msg1 = CompressedImage()
	msg1.format = "jpeg"
	msg1.header.stamp = rospy.Time.now()
	msg1.data = np.array(cv2.imencode('.jpg', img_show)[1]).tostring()
	pub.publish(msg1)

def diff(x1,y1,x2,y2):
	return  pow(pow(abs(x1-x2),2)+pow(abs(y1-y2),2),0.5)

def scan_cubic():
	r = 1
	lst = []
	for x1,y1 in zip(x_out,y_out):
		tmp = []
		for x2,y2,r,t in zip(x_out,y_out,r_out,theta_out):
			dis = diff(x1,y1,x2,y2)
			if dis <= r:
				tmp.append((x2,y2,r,t,dis))
		lst.append(tmp)
	def cmp(val):
		a = len(val) #len is amount of objects
		b = 0 #initial
		for i in val: 
			b+=i[4]
		b/=a #average of dis
		return a, -b
	if not lst: return []
	ans = max(lst,key=cmp)
	return ans[:4]

def Process():
	global Img_frame, preImg_gray, Img_show, count, p0, p1, good_new, good_old, r_met, theta_met, status, mask, new_pos, old_pos
	global r_out, theta_out
	new_pos, old_pos = [], []
	last_frame = False
	res = sonar_cubicmsg()  

	while(Img_frame is None):
		print "Img_frame: None"
		rospy.sleep(0.01)
		continue
	
	Img_gray = cv2.cvtColor(Img_frame, cv2.COLOR_BGR2GRAY)
	p0 = findobject(Img_gray)
	#r_out,theta_out = Tracking(p0)

	res.r = []
	res.theta = []
	res.x = []
	res.y = []
	ans = scan_cubic()
	for i in ans:
		res.r.append(i[2])
		res.theta.append(i[3])
		res.x.append(i[0])
		res.y.append(i[1])

	res.status = status
	
	if p0 == (-1,-1) or not res.r :res.status = False
	else : res.status = True

	print res.r
	print res.theta
	print res.status
	print res.x
	print res.y
	
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
	subTopic = "/connect_image_sonar"
	sub = rospy.Subscriber(subTopic, Image, image_callback,  queue_size = 1)
	rospy.Service('/sonar_image', sonar_cubicsrv, tracking_callback)
	print("finished setup")
	rospy.spin()
