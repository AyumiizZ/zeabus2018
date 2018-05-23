#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
import math
from robosub_qualifying_lib import *
from zeabus_example.msg import robosub_path_msg
from zeabus_example.srv import robosub_path_srv
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
img = None
img_res = None
sub_sampling = 1


def mission_callback(msg):
    print_result('mission_callback')
    task = msg.task.data

    print('task:', str(task))
    if task == 'path':
        return find_path()


def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    # img = cv.resize(img,(1128,874))
    img_res = img.copy()


def get_obj(img):
    '''
    get mask obj
    '''
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower = np.array([20, 140, 0], dtype=np.uint8)
    upper = np.array([62, 255, 255], dtype=np.uint8)
    # lower = np.array([0, 0, 0], dtype=np.uint8)
    # upper = np.array([255, 255, 255], dtype=np.uint8)
    mask = cv.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv.GaussianBlur(mask, (5, 5), 0)
    mask = cv.erode(mask, kernel)
    mask = cv.erode(mask, kernel)
    mask = cv.dilate(mask, kernel)
    mask = cv.dilate(mask, kernel)
    return mask


def get_cx(mask):
    global img
    himg, wimg = img.shape[:2]
    cv.line(img, (0, himg/2), (wimg, himg/2), (0, 255, 0), 10)
    cx = []
    cy = []
    sum_area = 0
    c = 0
    sli = 8
    for i in range(sli):
        a = ((himg/2)*i/sli)
        b = ((himg/2)*(i+1)/sli)
        ROI = mask[a:b, ]
        # cv.imshow('temp',ROI)
        cnt = cv.findContours(ROI, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        if len(cnt) >= 1:
            cnt = max(cnt, key=cv.contourArea)
            this_area = cv.contourArea(cnt)
            if this_area > 4000:
                M = cv.moments(cnt)
                ROI_cx = int(M["m10"]/M["m00"])
                ROI_cy = int(M['m01']/M['m00']) + a
                if ROI_cx >= 0.05 * wimg and ROI_cx <= 0.95 * wimg:
                    cv.circle(img, (ROI_cx, ROI_cy), 10, (255, 0, 0), -1)
                    cx.append(ROI_cx)
                    cy.append(ROI_cy)
                    sum_area += this_area
                    c += 1
    avg_area = 0 if c == 0 else sum_area/c
    return cx[::-1], cy[::-1], avg_area


def find_angle(cx, cy):
    deg = []
    temp = []
    for i in range(len(cx)-1):
        rad = math.atan2(cy[i+1]-cy[i], cx[i+1]-cx[i])
        this_deg = math.degrees(rad)
        if len(deg) > 0 and abs(this_deg-deg[-1]) > 5:
            cv.circle(img, (cx[i], cy[i]), 10, (255, 255, 255), -1)
            break
        deg.append(this_deg)
    return abs(sum(deg)/len(deg))-90

# message_path(cx=-1, cy=-1, area=-1, degree=-999, appear=False):


def find_path():

    while img is None and not rospy.is_shutdown():
        print('img is none.\nPlease check topic name or check camera is running')
    mask = get_obj(img)
    cx, cy, area = get_cx(mask)
    appear = len(cx) > 0
    publish_result(img, 'bgr', '/path/img')
    publish_result(mask, 'gray', '/path/mask')
    if len(cx) > 0:
        himg, wimg = img.shape[:2]
        return_cx = 1.0*(cx[0] - (himg/2))/(1.0*himg/2)
        return_cy = 1.0*(cy[0] - (wimg/2))/(1.0*wimg/2)
        return_area = (1.0*area*16)/(himg*wimg)
        cv.circle(img, (cx[0], cy[0]), 10, (255, 0, 0), -1)
        if len(cx) > 1:
            degrees = find_angle(cx, cy)
            return message_path(cx=return_cx, cy=return_cy, area=return_area, degrees=degrees, appear=True)
        return message_path(cx=return_cx, cy=return_cy, area=return_area, appear=True)
    return message_path()


if __name__ == '__main__':
    rospy.init_node('vision_path', anonymous=True)
    # image_topic = "/syrena/front_cam/image_raw/compressed"
    image_topic = "/bottom/left/image_raw/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print "init_pub_sub"
    rospy.Service('vision_path', robosub_path_srv(),
                  mission_callback)
    print "init_ser"
    rospy.spin()
