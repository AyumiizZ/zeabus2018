#!/usr/bin/python2.7

import math
import rospy
import cv2 as cv
import numpy as np
from vision_lib import *
from sensor_msgs.msg import CompressedImage

img = None
img_res = None
sub_sampling = 0.2
image_topic = '/top/center/image_raw/compressed'
# image_topic = '/stereo/right/image_rect_color/compressed'
publish_topic = '/vision/dice/'


def mission_callback(msg):
    print_result('mission_callback')
    task = msg.task.data
    print('task:', str(task))
    if task == 'dice':
        return find_dice()


def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()


# def convolution(img,kernel_size=3):
#     lab = cv.cvtColor(img,cv.COLOR_BGR2Lab)
#     l,a,b = cv.split(lab)
#     r,c = l.shape
#     result = np.zeros((r,c),np.uint8)
#     size = int(kernel_size/2)
#     for i in range(kernel_size,r-kernel_size):
#         for j in range(kernel_size,c-kernel_size):
#             avg = np.mean(l[i-size:i+size,j-size:j+size])
#             avg = min(127,avg)
#             result[i,j] = np.uint8(avg)

#     result = cv.merge((result,a,b))
#     result = cv.cvtColor(result,cv.COLOR_Lab2BGR)
#     return result

def convolution(img,kernel_size=3):
    hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)
    r,c = v.shape
    kernel = np.ones((kernel_size,kernel_size),np.float32)/(kernel_size*kernel_size)
    dst = cv.filter2D(v,-1,kernel)
    result = np.zeros((r,c),np.uint8)
    # size = int(kernel_size/2)

    # for i in range(kernel_size,r-kernel_size):
    #     for j in range(kernel_size,c-kernel_size):
    #         # avg = np.mean(v[i-size:i+size,j-size:j+size])
    #         avg = min(150,dst[i,j])
    #         result[i,j] = np.uint8(avg)
    v[v > 127] = 127
    result = cv.merge((h,s,v))
    result = cv.cvtColor(result,cv.COLOR_HSV2BGR)
    result = cv.resize(result,(0,0),fx=2.5,fy=2.5)
    return result

def convolution_brightness():
    global img, publish_topic

    while not rospy.is_shutdown():
        if img is None:
            continue
        conv = convolution(img,3)
        clahe_img = clahe(conv)
        publish_result(conv, 'bgr', publish_topic + 'result')
        publish_result(clahe_img, 'bgr', publish_topic + 'clahe')

        # cv.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('vision_convolution_brightness', anonymous=False)
    print_result("INIT NODE")

    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")

    convolution_brightness()