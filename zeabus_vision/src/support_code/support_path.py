#!/usr/bin/python2.7

import math
import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('zeabus_vision')+'/src')
from vision_lib import *
import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import CompressedImage


img = None
img_res = None
sub_sampling = 0.25
pub_topic = "/vision/path/"

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)

def find_triangle(mask):
    _, mask = cv.threshold(mask, 20, 255, cv.THRESH_BINARY)
    hierarchy, contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    count = 0
    res = cv.merge((mask,mask,mask))
    cv.drawContours(res,contours,-1,(0,255,255),1)

    for cnt in contours:
        peri = cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, 0.01 * peri, True)
        # cv.drawContours(res,approx,-1,(255,0,0),1)
        if len(approx) == 3:
            count += 1
            cv.drawContours(res,cnt,-1,(0,0,255),2)
    cv.imshow('mask2',res)
    cv.imshow('mask3',mask)
    cv.waitKey(1)
    return count

def is_full_path(mask):
    tmp = mask.copy()
    tmp.fill(0)
    _, mask = cv.threshold(mask, 20, 255, cv.THRESH_BINARY)
    _
    hierarchy, contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    cv.drawContours(tmp,contours,-1,(255,255,255),1)
    count = find_triangle(tmp)
    print(count)
    cv.imshow('mask1',mask)
    cv.imshow('res1',tmp)
    cv.waitKey(1)
    if 2 <= count <= 4:
        return True,0,0,0
    else:
        return False,0,0,0


def is_half_path(mask):
    path_w = 30
    # 30/2 = 15
    path_h = 6
    pass
    return False,0,0,0
    

def find_path():
    global img, img_res
    
    area_ratio_upper = 0.6
    area_ratio_lower = 0.35

    lowerb = np.array([0,7,29],np.uint8) 
    upperb =  np.array([63,211,243],np.uint8)

  
    while not rospy.is_shutdown():
        if img is None:
            continue
        is_path = False
        cx,cy = -1,-1
        res = img.copy()
        hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
        
        mask = cv.inRange(hsv,lowerb,upperb)
        erode = cv.erode(mask,get_kernel('rect',(3,3)))
        dilate = cv.dilate(erode,get_kernel("rect",(7,7)))
        _, dilate = cv.threshold(dilate, 20, 255, cv.THRESH_BINARY)
        
        hierarchy, contours, _ = cv.findContours(dilate, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
        
        for cnt in contours:
            type_path = 'Not'
            rect = cv.minAreaRect(cnt)
            (x,y), (width, height), angle = rect
            area_cnt = cv.contourArea(cnt)
            if area_cnt <= 1000:
                x,y = int(x),int(y)
                width,height = int(width), int(height)
                cv.rectangle(dilate,(x-int(width/2),y-int(height/2)),(x+width,y+height),(0,0,0),-1)
                continue
            area_box = width * height
            area_ratio = area_cnt / area_box

            xx,yy,ww,hh = cv.boundingRect(cnt)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(res,[box],0,(0,0,255),1)
    
            if area_ratio_lower <= area_ratio <= area_ratio_upper:
                roi = mask.copy()
                roi.fill(0)
                roi1 = roi.copy()
                cv.drawContours(roi,[box],0,(255,255,255),-1)
                cv.drawContours(roi1,[box],0,(255,255,255),-1)
                roi = roi & dilate
                roi = roi ^ roi1
                roi[:5,:] = 0
                roi[-5:,:] = 0
                roi = cv.erode(roi,get_kernel('rect',(5,5)))
                is_path, cx, cy, _ = is_full_path(roi)
                if is_path:
                    angle -= 22.5
                    rect = (x,y), (2, int(height/2.0)), angle 
                    box = cv.boxPoints(rect)
                    box = np.int0(box) 
                    cv.drawContours(res,[box],0,(0,255,255),2)
                    cv.circle(res,(int(x),int(y)),3,(0,255,0),-1)
                    type_path = 'Full'
            elif area_ratio > area_ratio_upper + 0.1:
                type_path = 'Half'
                is_path, cx, cy, angle = is_half_path(roi)
            angle = math.radians(-angle)
            print(area_ratio)
            font = cv.FONT_HERSHEY_SIMPLEX
            center = (int(x),int(y))
            if not type_path == 'Not':
                cv.putText(res, type_path + '_' + str("%.2f" % angle) + '_' + str("%.2f" % area_ratio), center, font, 0.5, (0,0,255), 1, cv.LINE_AA)
            
        # cv.imshow('img',img)
        cv.imshow('res',res)
        cv.imshow('dilate',dilate)

        k = cv.waitKey(1) & 0xFF
        if k == ord('q'):
            break
    
    plt.show()
    
if __name__ == '__main__':
    image_topic = '/bottom/left/image_raw/compressed'
    rospy.init_node('vision_path', anonymous=False)
    print("INIT NODE")
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print("INIT SUBSCRIBER")
    find_path()
    plt.close()