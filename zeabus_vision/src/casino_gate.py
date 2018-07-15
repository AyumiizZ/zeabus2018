#!/usr/bin/python2.7
"""
    To use in simulator please assign world varible (line 20) to 'sim' 
    # world = 'sim'
    To use in real world please assign world varible (line 20) to 'real' 
    # world = 'real'
"""
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from zeabus_vision.msg import vision_casino_gate
from zeabus_vision.srv import vision_srv_casino_gate
from cv_bridge import CvBridge, CvBridgeError
from vision_lib import *
img = None
img_res = None
sub_sampling = 1
pub_topic = "/vision/casino_gate/"
world = "real"


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    print_result('mission_callback')

    task = msg.task.data

    print('task:', str(task))
    if task == 'casino_gate':
        return find_gate()


def image_callback(msg):
    """
        Convert data from camera to image
    """
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    size = 500
    r = 1.0*size / img.shape[1]
    dim = (size, int(img.shape[0] * r))
    resized = cv.resize(img, dim, interpolation=cv.INTER_AREA)
    img = resized
    img_res = img.copy()


def message(cx1=-1, cx2=-1, area=-1, appear=False):
    """
        Convert value into a message (from vision_qualifying_gate.msg)
        Returns:
            vision_qualifying_gate (message): a group of value from args
    """
    m = vision_casino_gate()
    m.cx1 = cx1
    m.cx2 = cx2
    m.area = area
    m.appear = appear
    print(m)
    return m


def rm_sure_bg(img):
    lower = np.array([0, 0, 0], dtype=np.uint8)
    upper = np.array([255, 100, 100], dtype=np.uint8)
    mask = cv.inRange(img, lower, upper)
    return mask


def get_object():
    """
        get mask from picture and remove some noise
        Returns:
            mask (ONLY GATE AREA)
    """
    global img
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # hsv_map = color_mapping(gray)
    hsv_map = cv.applyColorMap(gray, cv.COLORMAP_HSV)
    lower = np.array([0, 90, 80], dtype=np.uint8)
    upper = np.array([180, 255, 255], dtype=np.uint8)
    not_bg = rm_sure_bg(img)
    map_mask = cv.inRange(hsv_map, lower, upper)
    if not_bg.shape[:2] != map_mask.shape[:2]:
        return map_mask
    all_mask = cv.bitwise_and(not_bg, map_mask)
    return all_mask
    # b,g,r = cv.split(img)
    # tc = 100
    # # hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    # canny = cv.Canny(b,tc,3*tc)
    # publish_result(canny, 'gray', pub_topic + 'canny')
    # kernal = cv.getStructuringElement(cv.MORPH_RECT,(10,10))
    # closed = cv.morphologyEx(canny,cv.MORPH_CLOSE,kernal)
    # publish_result(closed, 'gray', pub_topic + 'closed')
    # return closed


def find_gate():
    """
        find gate on the picture and draw rectangle cover the gate

        Returns:
            message: (a group of data from vision_qualifying_gate.msg)
            if cannot find gate:
                return default value
            else if find a gate (a little bit noise):
                return float: cx in range of -1 to 1, 
                       int: pos 
                              0 = all part of gate (can calculate cx)
                             -1 = left side of gate
                              1 = right side of gate
                            -99 = not sure (can find only one pole of gate)
                       float: area in range of 0 to 1,
                       bool: appear is true
            else if find a gate (a lot of noise):
                return float: cx in range of -1 to 1, 
                       int: pos is 0
                       float: area in range of 0 to 1,
                       bool: appear is true         
    """
    global img, img_res
    while img is None and not rospy.is_shutdown():
        print('img is none.\nPlease check topic name or check camera is running')
    area = -1
    appear = False
    himg,wimg = img.shape[:2]
    mask = get_object()
    publish_result(mask, 'gray', pub_topic+'mask')
    # return message()
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    ROI = []
    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)
        area = cv.contourArea(cnt)
        w_h_ratio = 1.0*w/h
        right_excess = (x+w > 0.95*wimg)
        left_excess = (x < 0.05*wimg)
        if area > 1000 and w_h_ratio <= 2.5 and w_h_ratio >= 1.5 and not right_excess and not left_excess:
            ROI.append(cnt)

    if ROI == []:
        mode = 1
    elif len(ROI) == 1:
        mode = 2
    elif len(ROI) > 1:
        mode = 3
    
    if mode == 1:
        return message()
    elif mode == 2:
        cnt = ROI[0]
        x, y, w, h = cv.boundingRect(cnt)
        cv.rectangle(img,(x,y),(x+w,y+h),(0,0,255))
        area = 1.0*w*h/(wimg*himg)
        return_cx1 = x+(w/4)
        return_cx2 = x+(w*3/4)
        return_y = y+(h/2)
        cv.circle(img,(return_cx1,return_y),3,(0,0,0),-1)
        cv.circle(img,(return_cx2,return_y),3,(0,0,0),-1)
        return_cx1 = Aconvert(return_cx1,wimg)
        return_cx2 = Aconvert(return_cx2,wimg)
        publish_result(img, 'bgr', pub_topic + 'img')
        return message(cx1=return_cx1,cx2=return_cx2,area=area,appear=True)
    elif mode == 2:
        cnt = max(ROI,key=cv.contourArea)
        x, y, w, h = cv.boundingRect(cnt)
        cv.rectangle(img,(x,y),(x+w,y+h),(0,0,255))
        area = 1.0*w*h/(wimg*himg)
        return_cx1 = x+(w/4)
        return_cx2 = x+(w*3/4)
        return_y = y+(h/2)
        cv.circle(img,(return_x1,return_y),3,(0,0,0),-1)
        cv.circle(img,(return_x2,return_y),3,(0,0,0),-1)
        return_cx1 = Aconvert(return_cx1,wimg)
        return_cx2 = Aconvert(return_cx2,wimg)
        publish_result(img, 'bgr', pub_topic + 'img')
        return message(cx1=return_cx1,cx2=return_cx2,area=area,appear=True)
    
    # # print "c1"
    # # print len(contours)
    # for cnt in contours:
    #     x,y,w,h = cv.boundingRect(cnt)
    #     if w < 100 or h < 100:
    #         continue
    #     w_h_ratio = 1.0*w/h
    #     # print w_h_ratio
    #     # cv.rectangle(img,(x,y),(x+w,y+h),(0,0,255))
    #     himg,wimg = img.shape[:2]
    #     right_excess = (x+w > 0.95*wimg)
    #     left_excess = (x < 0.05*wimg)
    #     if not right_excess and not left_excess:
    #     # if w_h_ratio <= 2.2 and w_h_ratio >= 1.8:
    #         appear = True
    #         ROI.append(cnt)
    #         cv.rectangle(img,(x,y),(x+w,y+h),(0,0,255),)
    #         area = 1.0*w*h/(wimg*himg)
    #         return_cx1 = x+(w/4)
    #         return_cx2 = x+(w*3/4)
    #         return_y = y+(h/2)
    #         cv.circle(img,(return_x1,return_y),3,(0,0,0),-1)
    #         cv.circle(img,(return_x2,return_y),3,(0,0,0),-1)
    #         return_cx1 = Aconvert(return_cx1,wimg)
    #         return_cx2 = Aconvert(return_cx2,wimg)
    #         publish_result(img, 'bgr', pub_topic + 'img')
    # return message(return_cx1,return_cx2,area,appear)


if __name__ == '__main__':
    rospy.init_node('vision_casino_gate', anonymous=False)
    print_result("INIT NODE")
    image_topic = get_topic("front", world)
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")
    rospy.Service('vision_casino_gate',
                  vision_srv_casino_gate(), mission_callback)
    print_result("INIT SERVICE")
    rospy.spin()
    print_result("END PROGRAM")
