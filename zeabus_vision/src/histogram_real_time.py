#!/usr/bin/python2.7

import math
import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from vision_lib import *
from sensor_msgs.msg import CompressedImage

img = None
img_res = None
sub_sampling = 0.5
image_topic = '/stereo/right/image_rect_color/compressed'
# image_topic = '/top/center/image_raw/compressed'
# image_topic = '/bottom/left/image_raw/compressed'

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)

def display():
    # global img, publish_topic

    img = cv.imread('/home/skconan/Downloads/red.jpg',1)
    while not rospy.is_shutdown():
        
        if img is None:
            print('image is None')
            continue
        hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
        Lab = cv.cvtColor(img,cv.COLOR_BGR2Lab)
        h,s,v = cv.split(hsv)
        L,a,b = cv.split(Lab)
        # v = L
        # print(v.min())
        histr = cv.calcHist([v],[0],None,[256],[0,256])
        plt.plot(histr,color = 'cyan')
        histr = cv.calcHist([s],[0],None,[256],[0,256])
        plt.plot(histr,color = 'pink')
        # plt.plot(v,color = 'yellow')
        # v = np.array(v,np.float)
        v -= v.min() 
        v /= (v.max()-v.min())
        v = np.uint8(v*100)
        v += 25
        # v[v<=26] = 50
        v[v>200] -= 26
    
        # v_mode = get_mode(v)
        # if v_mode is None:
        #     v_mode = 10
        # v_mode = np.uint8(v)

        # for i in range(1,20):
        #     v -= 1
        #     v_mode = get_mode(v)
        #     if v_mode is None:
        #         v_mode = 10
        #     if v_mode <= 10:
        #         break
        # s -= s.min() 
        # s /= (s.max()-s.min())
        # s = equalization_gray(s)
        # s = np.array(s,np.float)
        # s /= 255.0
        # s = np.uint8(s*100)
        # s += 100

        hsv_result = cv.merge((h,s,v))
        # Lab_result = cv.merge((v,a,b))
        result = cv.cvtColor(hsv_result,cv.COLOR_HSV2BGR)
        # result = cv.cvtColor(Lab_result,cv.COLOR_Lab2BGR)
        publish_result(result,'bgr','result')
        histr = cv.calcHist([v],[0],None,[256],[0,256])
        plt.plot(histr,color = 'blue')
        histr = cv.calcHist([s],[0],None,[256],[0,256])
        plt.plot(histr,color = 'red')
        plt.xlim([0,256])
        plt.pause(0.05)
        plt.clf()
    plt.close()
    # plt.show()

if __name__ == '__main__':
    rospy.init_node('vision_dice', anonymous=False)
    print_result("INIT NODE")

    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")

    display()
    # rospy.spin()
