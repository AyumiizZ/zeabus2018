#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
import math
from sensor_msgs.msg import CompressedImage
from zeabus_vision.msg import vision_path
from zeabus_vision.srv import vision_srv_path
from vision_lib import *
img_bot = None
img_bot_res = None
himg = -1
wimg = -1
sub_sampling = 1
publish_topic = "/vision/path"


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    print_result('mission_callback', color_text.CYAN)

    task = msg.task.data

    print('task:', str(task))
    if task == 'path':
        return find_path()


def image_callback(msg):
    """
        Convert data from camera to image
    """
    global img_bot, sub_sampling, img_bot_res, himg, wimg
    arr = np.fromstring(msg.data, np.uint8)
    img_bot = cv.resize(cv.imdecode(arr, 1), (0, 0),
                        fx=sub_sampling, fy=sub_sampling)
    himg, wimg = img_bot.shape[:2]
    himg = int(himg/3)
    wimg = int(wimg/3)
    img_bot = cv.resize(img_bot, (wimg, himg))
    img_bot_res = img_bot.copy()


def message(cx=-1, cy=-1, area=-1, degrees=0, appear=False):
    """
        Convert value into a message (from vision_path.msg)
        Returns:
            vision_path (message): a group of value from args
    """
    m = vision_path()
    m.cx = cx
    m.cy = cy
    m.area = area
    m.degrees = degrees
    m.appear = appear
    print(m)
    return m


def get_mask(img):
    """
        get mask from picture
        Returns:
            mask from range
    """
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    upper, lower = get_color_range('orange', 'bottom', '1', 'path')
    mask = cv.inRange(hsv, lower, upper)
    return mask


def get_cx_cy(mask):
    global himg, wimg,img_bot_res
    cm = []
    sum_area = 0
    num_slice = 8
    for i in range(num_slice):
        begin = int(int(himg/2)*i/num_slice)
        end = int(int(himg/2)*(i+1)/num_slice)
        slice_mask = mask[begin:end, ]
        contours = cv.findContours(
            slice_mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[1]
        if len(contours) >= 1:
            cnt = max(contours, key=cv.contourArea)
            area = cv.contourArea(cnt)
            if area > 200:
                x, y, w, h = cv.boundingRect(cnt)
                cx = float(x+w)/2
                cy = float(y+h)/2
                if cx >= 0.05 * wimg and cx <= 0.95 * wimg:
                    cv.circle(img_bot_res, (cx, cy), 3, (255, 0, 0), -1)
                    cm.append(Points(cx=cx, cy=cy,himg=himg,wimg=wimg))
                    sum_area += area
    cm = sorted(cm, key=lambda x: x.cy, reverse=True)
    average_area = float(sum_area)/len(cm) if len(cm) > 0 else 0
    return cm, average_area


def find_angle(cm):
    global img_bot_res
    if len(cm) < 2:
        return 0
    degrees_between_point = []
    cx,cy = cm
    for i in range(len(cm)-1):
        rad = math.atan2()
    pass

def find_path():
    pass
