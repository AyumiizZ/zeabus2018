#!/usr/bin/python2.7

import math
from vision_lib import *
import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import CompressedImage


img = None
img_res = None
sub_sampling = 0.5
pub_topic = "/vision/path/"
world = "real"



def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)

def get_mode(data):
    if len(data.shape) > 1:
        data = data.ravel()
    

    count = np.bincount(data)
    if len(count) > 200:
        count = count[:200]
    max = count.max()
    count = list(count)
    return count.index(max)

def trimmed(data, trimmedValue):
        data = list(data)
        for i in range(0, trimmedValue[0]):
            data = filter(lambda a: a != i, data)
        for i in range(trimmedValue[1], 256):
            data = filter(lambda a: a != i, data)
        return data

def find_path():
    global img, img_res
    lowerb = np.array([0,0,31],np.uint8) 
    upperb =  np.array([69,190,150],np.uint8)

  
    while not rospy.is_shutdown():
        if img is None:
            continue
        img = cv.GaussianBlur(img, (3, 3), 0)
        hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv,lowerb,upperb)
        erode = cv.erode(mask,get_kernel('rect',(3,3)))
        dilate = cv.dilate(erode,get_kernel("rect",(7,7)))
        # dilate = cv.dilate(dilate,get_kernel("/",(7,7)))


        cv.imshow('img',img)
        cv.imshow('mask',mask)
        cv.imshow('dilate',dilate)
        cv.imshow('hsv',hsv)

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